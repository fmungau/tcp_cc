 //===========================================================================
// Name        : SureLinkXG_tcp_bbr.cpp
// Author      : Frank M
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include "threadprocessor.h"

alignas(64) int debug_tcp_bbr = 0;

alignas(64) int ref_fid = 0;

/* Cut the cwnd to a fraction of the BDP during PROBE_RTT.
 *If this value is zero, cut cwnd to 4 packets when entering PROBE_RTT.
 */
int    bbr_bdp_fraction = 2;

// Pls see pg 21 of the bbr rfc
void bbr_init(flow_info* fs_ptr, double t_evt)
{
	if (debug_tcp_bbr)//(1)//
		cout << "Initializing TCP BBR Connection..." << endl;

	int fid;

	// get fid
	fid = fs_ptr->fid;

	fs_ptr->C_delivered              = 0;    
	fs_ptr->BBR_BtlBw                = 0.0;  

	fs_ptr->C_next_send_time         = 0.0;  // stop pacing

	fs_ptr->BBR_RTprop               = (fs_ptr->t_srtt > 0) ? fs_ptr->t_srtt : t_inf; // different in linux vs pg 21 bbr rfc
	fs_ptr->BBR_rtprop_stamp         = t_evt;  
	fs_ptr->BBR_ProbeRTT_round_done  = 0;    
	fs_ptr->BBR_packet_conservation  = 0;    
	fs_ptr->BBR_prior_cwnd           = 0;    

	// BBRInitRoundCounting(), pg 11 bbr rfc 
	fs_ptr->BBR_next_round_delivered = 0; 
	fs_ptr->BBR_round_start          = 0; 
	fs_ptr->BBR_round_count          = 0; 

	// BBRInitFullPipe(), pg 23 bbr rfc
	fs_ptr->BBR_filled_pipe          = 0; 
	fs_ptr->BBR_fullBw_count         = 0; 
	fs_ptr->BBR_full_BtlBw           = 0;

	fs_ptr->max_packets_out          = 0;

	// BBREnterStartup(), pg 22 bbr rfc
	fs_ptr->BBR_mode                 = BBR_STARTUP; 
	fs_ptr->BBR_pacing_gain          = bbr_high_gain;
	fs_ptr->BBR_cwnd_gain            = bbr_high_gain;

	// BBRInitPacingRate(), pg 15 bbr rfc
	bbr_set_pacing_rate(fs_ptr);

	// init the random number generator
	//srand(time(0));

	// Init the Rate Sampling (specific to WANAI code)
	fs_ptr->rs_prior_delivered       = 0;
	fs_ptr->rs_prior_time            = 0.0;

	fs_ptr->rs_delivery_rate         = 0.0;

	// init the windowed minmax filter (BBR sender specific)
	init_bbr_minmax(fid);
	
	if (debug_tcp_bbr)//(1)//
		std::cout << "Done!!" << endl;
}


// this is bbr_main in the tcp_bbr.c file, and pg 8 of bbr rfc
void bbr_update_on_ack(flow_info* fs_ptr, seg_info* sh_ptr, snd_info* sv_ptr, double t_evt)
{
	// update the bdp model
	bbr_update_model(fs_ptr, sh_ptr, sv_ptr, t_evt); 

	// BBRUpdateControlParameters()
	bbr_set_pacing_rate(fs_ptr);

	//bbr_set_send_quantum(); // excluded here,... BBR.sendquantum calculated elsewhere as bbr_tso_segs_goal
	bbr_set_cwnd(fs_ptr, sh_ptr, sv_ptr, t_evt);
}


// On every ACK, the BBR algorithm executes the following
// BBRUpdateOnACK() steps in order to update its network path model,
// update its state machine, and adjust its control parameters to adapt
// to the updated model, pg 8 bbr rfc
void bbr_update_model(flow_info* fs_ptr, seg_info* sh_ptr, snd_info* sv_ptr, double t_now)
{
	if (debug_tcp_bbr)
		cout << "1. Updating the BBR Model" << endl;

	bbr_update_bw(fs_ptr, sh_ptr, sv_ptr, t_now);
	bbr_update_cycle_phase(fs_ptr, sh_ptr, t_now);
	bbr_check_full_bw_reached(fs_ptr);
	bbr_check_drain(fs_ptr, sv_ptr, t_now);
	bbr_update_min_rtt(fs_ptr, sh_ptr, sv_ptr, t_now);
	bbr_update_gains(fs_ptr);

}


// Estimate the bandwidth based on how fast packets are delivered 
// see pg 12 bbr rfc
void bbr_update_bw(flow_info* fs_ptr, seg_info* sh_ptr, snd_info* sv_ptr, double t_evt)
{
	if (debug_tcp_bbr)
		cout << "	A. --- Updating the BBR.BtlBw... ---" << endl;

	// init vars
	int f_id = sh_ptr->flowId;

	// pg 26 rfc, "prior_inflight" is the amount of data that was in flight before processing this ACK.
	int prior_inflight;

	double rs_delivery_rate;

	// get prior in_flight
	if (sh_ptr->ack_seq > 2)
		prior_inflight = fs_ptr->prior_in_flight;
	else
		prior_inflight = 1;

	if (debug_tcp_bbr)
		cout << "Current BBR BtlBw: " << fs_ptr->BBR_BtlBw << " Bytes per second, "
		     << "Sampled Rate: " << sv_ptr->rs_delivery_rate << " Bytes per second " << endl;

	fs_ptr->BBR_round_start = 0; // clear

	if (debug_tcp_bbr)
		cout << "??? Checking BBR next round delivered " << " - "
		<< "ACK P_delivered: " << sh_ptr->P_delivered << " Segs ??? "
		<< "C Delivered Round Limit: " << fs_ptr->BBR_next_round_delivered << " Segs ???" << endl;

	// check if we have reached the next RTT 
	// see BBRUpdateRound(), pg 11 bbr rfc,  
	if (sh_ptr->P_delivered >= fs_ptr->BBR_next_round_delivered)
	{
		if (debug_tcp_bbr)
			cout << "  i. Executing BBRUpdateRound() - BBR Filter for variations in BtlBw" << endl;

		fs_ptr->BBR_next_round_delivered = sv_ptr->C_delivered;	// Segs, not Bytes... tcp_socket tp translates to C
		fs_ptr->BBR_round_count++;
		fs_ptr->BBR_round_start = 1;
		fs_ptr->BBR_packet_conservation = 0; // this is in linux		

		//if (fs_ptr->BBR_mode != BBR_PROBE_RTT) // usefull to debug synchronisation	 
			log_cwnd(f_id, t_evt, prior_inflight);//fs_ptr->snd_cwnd//fs_ptr->BBR_BtlBw

		if (debug_tcp_bbr)//(1)//
			cout << "ooo Plotting CWND of " << fs_ptr->snd_cwnd << " SEGS, "
			<< "@ time: " << t_evt << " SECS ooo" << endl;

		double bbr_gamma = fs_ptr->t_srtt / (fs_ptr->t_srtt + fs_ptr->BBR_RTprop);

		/*if (f_id == ref_fid)
			cout << "bbr gamma: " << bbr_gamma << ", "
			<< "TCP SRTT: " << fs_ptr->t_srtt << ", "
			<< "BBR RTprop: " << fs_ptr->BBR_RTprop << endl;*/

		/*if (f_id == ref_fid)
			cout << "TIME: " << t_evt << ", "
			<< "BBR MODE: " << fs_ptr->BBR_mode << ", "
			<< "BBR PHASE: " << fs_ptr->BBR_GainCycleIdx << ", "
			<< "BBR BTLBw: " << fs_ptr->BBR_BtlBw << ", "
			<< "BBR Rate: " << sv_ptr->rs_delivery_rate << ", "
			<< "BBR RTprop: " << fs_ptr->BBR_RTprop << ", " 
			<< "TCP SRTT: " << fs_ptr->t_srtt << ", "
			<< "SND CWND: " << fs_ptr->snd_cwnd << ", "
			<< "PIPE: " << fs_ptr->Pipe << endl;*/
	}

	if (debug_tcp_bbr)
		cout << "BBR Round Start (0 - no, 1 - yes): " << fs_ptr->BBR_round_start << ", "
		     << "BBR Round count: " << fs_ptr->BBR_round_count << ", "
		     << "BBR Application Limited (0 - no, 1 - yes): " << fs_ptr->C_app_limited << endl;

	if (debug_tcp_bbr)
		cout << "??? Checking the measurement interval " << " - "
		<< "rs_interval: " << sv_ptr->rs_interval << " secs ??? "
		<< "BBR_RTprop: " << fs_ptr->BBR_RTprop << " secs ???" << " secs ??? "
		<< "BtlBw test (0 - FAIL, 1- YES)?: " << (sv_ptr->rs_interval < fs_ptr->BBR_RTprop) << " secs ???" << endl;

	// Normally we expect interval_us >= min_rtt. Note that rate may still be over-estimated when a spuriously
	// retransmistted skb was first (s)acked because "interval_us" is under-estimated (up to an RTT). However continuously
	// measuring the delivery rate during loss recovery is crucial for connections suffer heavy or prolonged losses.
	if (sv_ptr->rs_interval < fs_ptr->BBR_RTprop)
	{
		rs_delivery_rate = fs_ptr->BBR_BtlBw; // bw update - NOK

		if (debug_tcp_bbr)
			cout << "??? OPT 1 - measured rate: " << rs_delivery_rate << " Bps" << endl;
	}
	else
	{
		rs_delivery_rate = sv_ptr->rs_delivery_rate; // bw update - OK

		if (debug_tcp_bbr)
			cout << "??? OPT 2 - measured rate: " << rs_delivery_rate << " Bps" << endl;
	}
	
	// Incorporate new sample into our max bw filter, this is like pg 12 bbr rfc
	// note:
	// BtlBwFilterLen: A constant specifying the length of the BBR.BtlBw max
	// filter window for BBR.BtlBwFilter, BtlBwFilterLen is 10 packet - timed
	// round trips
	if ((rs_delivery_rate >= fs_ptr->BBR_BtlBw) || (fs_ptr->C_app_limited < 1))
	{
		if (debug_tcp_bbr)
			cout << "  ii. Executing BBRUpdateBtlBw() - BBRs estimate of the bandwidth available" << endl;

		/*cout << "BBR Rate: " << rs_delivery_rate << " Bps, "
			<< "BBR BtlBw: " << fs_ptr->BBR_BtlBw << " Bps" << endl;*/

		double time    = (double)fs_ptr->BBR_round_count;
		double value   = rs_delivery_rate; // in bytes per second
		int window_len = fs_ptr->BBR_BtlBwFilterLen;

		fs_ptr->BBR_BtlBw = minmax_running_max(f_id, time, value, window_len);

		if (debug_tcp_bbr)
			cout << "Updated BBR BtlBw: " << fs_ptr->BBR_BtlBw << " Bytes per second" << endl;
	}
}


// Gain cycling: cycle pacing gain to converge to fair share of available bw.
// see page 26 bbr rfc
// the is like BBRCheckCyclePhase()
void bbr_update_cycle_phase(flow_info* fs_ptr, seg_info* sh_ptr, double t_evt)
{
	// init vars
	int f_id = sh_ptr->flowId;

	if (debug_tcp_bbr)
		cout << "	B. --- Updating the BBR Cycle Phase ---" << endl;

	if (debug_tcp_bbr)
		cout << "@@@ Current Cycle Phase: " << fs_ptr->BBR_GainCycleIdx << "; "
		<< "Is it a new Cycle Phase? " << bbr_is_next_cycle_phase(fs_ptr, t_evt) << endl;

	if ((fs_ptr->BBR_mode == BBR_PROBE_BW) && (bbr_is_next_cycle_phase(fs_ptr, t_evt)))
	{
		if (debug_tcp_bbr)
			cout << "Updating the current cycle phase... " << endl;

		// BBRAdvanceCyclePhase()
		if (fs_ptr->BBR_GainCycleIdx + 1 < CYCLE_LEN)
			fs_ptr->BBR_GainCycleIdx++;
		else
			fs_ptr->BBR_GainCycleIdx = 0;

		fs_ptr->BBR_GainCycle_stamp = t_evt;

		/*if (f_id == ref_fid)
			cout << "TIME: " << t_evt << ", "
			<< "BBR MODE: " << fs_ptr->BBR_mode << ", "
			<< "BBR PHASE: " << fs_ptr->BBR_GainCycleIdx << ", "
			<< "BBR BTLBw: " << fs_ptr->BBR_BtlBw << ", "
			<< "BBR Rate: " << fs_ptr->rs_delivery_rate << ", "
			<< "BBR RTprop: " << fs_ptr->BBR_RTprop << ", "
			<< "TCP SRTT: " << fs_ptr->t_srtt << ", "
			<< "SND CWND: " << fs_ptr->snd_cwnd << ", "
			<< "PIPE: " << fs_ptr->Pipe << endl;*/

		if (debug_tcp_bbr)
			cout << "fs_ptr->BBR_GainCycleIdx: " << fs_ptr->BBR_GainCycleIdx << ", "
			<< "fs_ptr->BBR_GainCycle_stamp: " << fs_ptr->BBR_GainCycle_stamp << endl;

		if (debug_tcp_bbr)
			cout << "Done! Cycle phase updated: " << fs_ptr->BBR_GainCycleIdx << endl;
	}
}



// End cycle phase if it's time and/or we hit the phase's in-flight target.
// see page 26 bbr rfc
bool bbr_is_next_cycle_phase(flow_info* fs_ptr, double t_now)
{
	// "prior_inflight" is the amount of data that was in flight before processing this ACK.
	int prior_inflight; // = fs_ptr->Pipe;

	// logic variables
	bool is_full_length;
	bool is_packet_lost;

	if (debug_tcp_bbr)
		cout << "time now: " << t_now << ", \n"
		<< "Gain Cycle time stamp: " << fs_ptr->BBR_GainCycle_stamp << ", \n"
		<< "difference: " << (t_now - fs_ptr->BBR_GainCycle_stamp) << ", \n"
		<< "RTprop measured: " << fs_ptr->BBR_RTprop << endl;

	// check if we are probing...
	is_full_length = (t_now - fs_ptr->BBR_GainCycle_stamp) > fs_ptr->BBR_RTprop;

	is_packet_lost = ((fs_ptr->recover > 0) || (fs_ptr->init_rto > 0));

	if (debug_tcp_bbr)
		cout << "??? is it a full cycle length (0 - false, 1 - true): " << is_full_length << endl;	

	if (fs_ptr->BBR_pacing_gain == 1)
		return is_full_length;

	// Here, "prior_inflight" is the amount of data that was in flight
	// before processing this ACK.
	prior_inflight = bbr_packets_in_net_at_edt(fs_ptr, t_now) ;

	/*if (debug_tcp_bbr)
	{
		if (fs_ptr->BBR_pacing_gain > 1)
			cout << "Packets in flight at EDT: " << bbr_packets_in_net_at_edt(fs_ptr->BBR_pacing_gain, t_now) << ", \n"
				<< "Packets in flight now: " << prior_inflight << endl;
		else
			cout << "Packets in flight at EDT: " << bbr_packets_in_net_at_edt(1.0, t_now) << ", \n"
			<< "Packets in flight now: " << prior_inflight << endl;
	}*/

	if (fs_ptr->BBR_pacing_gain > 1)
		return (is_full_length && (is_packet_lost ||
		(prior_inflight >= bbr_target_cwnd(fs_ptr, t_now, 1.25))));// bbr_packets_in_net_at_edt(fs_ptr->BBR_pacing_gain, t_now))));

	// else... (BBR.pacing_gain < 1)
	return (is_full_length || (prior_inflight <= bbr_target_cwnd(fs_ptr, t_now, 1.0)));// (prior_inflight <= bbr_packets_in_net_at_edt(1.0, t_now)));
}



//
// This allows us to estimate Pipe when pacing
//
// With pacing at lower layers, there's often less data "in the network" than
// "in flight". With TSQ and departure time pacing at lower layers(e.g.fq),
// we often have several skbs queued in the pacing layer with a pre - scheduled
// earliest departure time(EDT)
//
// in_network_at_edt = inflight_at_edt - (EDT - now) * bw
//
// see BBRInflight pg 17 BBR RFC 
int bbr_packets_in_net_at_edt(flow_info* fs_ptr, double t_evt)
{
	int enode = 1;

	double delta_t, edt, mss_bytes, BtlBw;
	int in_network_at_edt, inflight_at_edt, edt_bdp_in_pkts;

	if (fs_ptr->BBR_RTprop == t_inf)
		return 1;

	// set bytes
	mss_bytes = (double)fs_ptr->mss_bytes;

	// set BtlBw
	BtlBw = fs_ptr->BBR_BtlBw;

	// find the earliest departure time (EDT)
	if (t_evt > fs_ptr->C_next_send_time)
		edt = t_evt;
	else
		edt = fs_ptr->C_next_send_time;

	// whats the time diff in the edt?
	delta_t = edt - t_evt;

	// (edt - t_evt) * fs_ptr->BBR_BtlBw / (double)(fs_ptr->mss_bytes);
	edt_bdp_in_pkts = (int)((delta_t * BtlBw) / mss_bytes);

	// (int)(gain * fs_ptr->BBR_BtlBw * fs_ptr->BBR_RTprop / (double)(fs_ptr->mss_bytes)); // fs_ptr->Pipe; 
	inflight_at_edt = fs_ptr->prior_in_flight; //fs_ptr->Pipe; // (int)bbr_bdp(enode, t_evt, gain);

	if (debug_tcp_bbr)
		cout << "EDT Packet in Flight: " << inflight_at_edt 
		     << ", EDT Left out: " << edt_bdp_in_pkts << endl;

	if (fs_ptr->BBR_pacing_gain > 1) // include seg goal
		inflight_at_edt += (int)bbr_tso_segs_goal(fs_ptr); // ?

	if (edt_bdp_in_pkts >= inflight_at_edt)
		return 0;

	// so edt_bdp_in_pkts packets will have left the network due to pacing at edt!!
	in_network_at_edt = inflight_at_edt - edt_bdp_in_pkts;

	if (debug_tcp_bbr)
		cout << "Packet in network at EDT: " << in_network_at_edt  << endl;

	return in_network_at_edt;
}


// Estimate when the pipe is full, using the change in delivery rate (pg 23 BBR RFC)
void bbr_check_full_bw_reached(flow_info* fs_ptr)
{
	//double bw_thresh;

	if (debug_tcp_bbr)
		cout << "	C. --- Checking for full BtlBw reached ---" << endl;

	if ((fs_ptr->BBR_filled_pipe > 0) || (fs_ptr->BBR_round_start < 1))
	{
		if (debug_tcp_bbr)
		{
			cout << "BBR Pipe filled (0 - NO, 1 - YES): " << fs_ptr->BBR_filled_pipe << "\n"
				<< "BBR Round Start (0 - NO, 1 - YES): " << fs_ptr->BBR_round_start << endl;

			cout << "...all done! " << endl;
		}

		return;
	}

	if (fs_ptr->BBR_BtlBw >= fs_ptr->BBR_full_BtlBw * 1.25)
	{
		fs_ptr->BBR_full_BtlBw = fs_ptr->BBR_BtlBw;
		fs_ptr->BBR_fullBw_count = 0;

		if (debug_tcp_bbr)
			cout << "BBR full BtlBw updated to: " << fs_ptr->BBR_full_BtlBw << " Bytes per sec" << endl;

		return;
	}

	// if another round w/o much growth (here we made it 1 round,.. no rwnd adjustment)
	if (++fs_ptr->BBR_fullBw_count >= 3)
		fs_ptr->BBR_filled_pipe = 1;

	if (debug_tcp_bbr)
		cout << "--> BBR fullBw Count: " << fs_ptr->BBR_fullBw_count << ", "
		     << "Pipe full (0 - NO, 1 - YES)? " << fs_ptr->BBR_filled_pipe << endl;
}


// If pipe is probably full, drain the queue and then enter steady-state.
//
// see BBRCheckDrain() pg 24 BBR RFC && torvalds linux
void bbr_check_drain(flow_info* fs_ptr, snd_info* sv_ptr, double t_now)
{
	// in_flight
	int    in_flight; 
	double packets_in_flight, bdp, max_packets_delivered;
	
	// set prior in flight
	if (fs_ptr->send_high == 0)
		in_flight = (fs_ptr->prior_in_flight - sv_ptr->packets_delivered);
	else
		in_flight = (fs_ptr->prior_in_flight - sv_ptr->packed_out);

	// in flight
	packets_in_flight = (double)in_flight;

	// bdp
	bdp = bbr_bdp(fs_ptr, t_now, 1.0);

	// max_packets after recovery
	max_packets_delivered = (double)fs_ptr->max_packets_delivered;
	
	if (debug_tcp_bbr)
		cout << "	D. --- BBR check drain ---" << "\n"
			 << "??? in_flight_at_edt: " << bbr_packets_in_net_at_edt(fs_ptr, t_now) << ", "
		     << "??? target_cwnd: " << bbr_target_cwnd(fs_ptr, t_now, 1.0) << ", "
			 << "??? bdp: " << bdp << ", "
		     << "??? in_flight: " << in_flight << ", "
		     << "??? max_packets_delivered: " << max_packets_delivered << endl;

	if (debug_tcp_bbr)
		cout << "??? Checking Full BW,... BBR filled pipe (0 - no, 1 -yes)?: " << fs_ptr->BBR_filled_pipe << endl;

	if ((fs_ptr->BBR_mode == BBR_STARTUP) && (fs_ptr->BBR_filled_pipe > 0))
	{
		if (debug_tcp_bbr)
			cout << "--> Entering DRAIN <-- " << endl;

		fs_ptr->BBR_mode = BBR_DRAIN;

		// WANAI BBR implementation transitions from START-UP to PROBE BW at FAST RE-TRANSMIT
		fs_ptr->snd_ssthresh = bbr_target_cwnd(fs_ptr, t_now, 1.0);

		if (debug_tcp_bbr)
			cout << "--> BBR MODE (0 - STARTUP, 1 - DRAIN, 2 - PROBE BW, 3 - PROBE RTT): " << fs_ptr->BBR_mode << endl;

	}

	// CHECK - (packets_in_flight <= max_packets_delivered))		
	if ((fs_ptr->BBR_mode == BBR_DRAIN) && (bbr_packets_in_net_at_edt(fs_ptr, t_now) <= bbr_target_cwnd(fs_ptr, t_now, 1.0)))
	{
		if (debug_tcp_bbr)
			cout << "--> Entering Probe BW <-- " << endl;

		fs_ptr->BBR_mode = BBR_PROBE_BW;

		if (debug_tcp_bbr)
			cout << "--> BBR MODE (0 - STARTUP, 1 - DRAIN, 2 - PROBE BW, 3 - PROBE RTT): " << fs_ptr->BBR_mode << endl;

		fs_ptr->BBR_GainCycle_stamp = t_now;

		// random init gain cycle index between 2 - 6 is ok, improves mixing and rairenes
		fs_ptr->BBR_GainCycleIdx = 1 + (rand() % 5); // 6; // (rand() % CYCLE_LEN);

		if (fs_ptr->BBR_GainCycleIdx + 1 < CYCLE_LEN)
			fs_ptr->BBR_GainCycleIdx++;
		else
			fs_ptr->BBR_GainCycleIdx = 0;

		if (debug_tcp_bbr)
			cout << "fs_ptr->BBR_GainCycleIdx: " << fs_ptr->BBR_GainCycleIdx << ", "
			<< "fs_ptr->BBR_GainCycle_stamp: " << fs_ptr->BBR_GainCycle_stamp << endl;
	}
}



// Find target cwnd. Right-size the cwnd based on min RTT and the
// estimated bottleneck bandwidth:
//
// cwnd = bw * min_rtt * gain = BDP * gain
//
// The key factor, gain, controls the amount of queue. While a small gain
// builds a smaller queue, it becomes more vulnerable to noise in RTT
// measurements (e.g., delayed ACKs or other ACK compression effects). This
// noise may cause BBR to under-estimate the rate.
// 
// see BBRUpdateTargetCwnd() pg 17 BBR RFC
int bbr_target_cwnd(flow_info* fs_ptr, double t_now, double cwnd_gain)
{
	int    twnd, enode = 1;
	double calc_twnd;

	if (fs_ptr->BBR_RTprop == t_inf)
		return 1;

	if (0)//(debug_tcp_bbr)
		cout << "--- Calculating the Target CWND ---" << endl;

	// calculate the bdp
	calc_twnd = bbr_bdp(fs_ptr, t_now, cwnd_gain);

	if (0)//(debug_tcp_bbr)
		cout << "Target CWND Est (pre-gain): " << calc_twnd << endl;

	//Apply the cwnd gain to the given value,
	calc_twnd = calc_twnd + (3 * bbr_tso_segs_goal(fs_ptr));

	// Reduce delayed ACKs by rounding up cwnd to the next even number.
	calc_twnd = (2 * ceil(calc_twnd / 2));

	// if Ensure gain cycling gets inflight above BDP even for small BDPs.
	if ((fs_ptr->BBR_mode == BBR_PROBE_BW) && (fs_ptr->BBR_GainCycleIdx == 0))
		calc_twnd += 2;

	twnd = (int)calc_twnd;

	if (debug_tcp_bbr)//(0)//
		cout << "BBR Target CWND (Segs - post gain): " << twnd << endl;

	return twnd;
}



// BBRSetSendQuantum() - On each ACK, BBR runs BBRSetSendQuantum() to update BBR.send_quantum
// See pg 16 bbr rfc
double bbr_tso_segs_goal(flow_info* fs_ptr)
{
	// NOTE: There are different units for pacing rate and BtlBw
	// check bbr_set_pacing_rate, pacing rate in bps 
	// fs_ptr->BBR_BtlBw in Bps!!
	double tso_segs, bytes;

	if (debug_tcp_bbr)//(0)//
		cout << "Calculating TSO Segs - BBR pacing rate: " << fs_ptr->BBR_pacing_rate << " Bps" << endl;

	// inititialised in bbr_init (Mbps)
	double pacing_rate = (fs_ptr->BBR_pacing_rate);//// (Mbps)

	double mss_bytes = (double)fs_ptr->mss_bytes;

	double min_rtt = (double)fs_ptr->BBR_RTprop * 1e3;	

	// Pacing rate in bytes per second, min_rtt (msec)
	if (pacing_rate < 150e3) // 1.2Mbps
		tso_segs = 1;
	else if (pacing_rate < 2e6) // 24Mbps
		tso_segs = 2;
	else
	{
		// bytes in 1 msec
		bytes = (pacing_rate * 0.001);

		/* tso burst size based on pacing rate */
		double segs_0 = (bytes / mss_bytes);

		// calculate max quantum (128 kBytes)
		double segs_1 = 64e3 / mss_bytes;

		// apply bound
		tso_segs = max(segs_1, segs_0);
	}

	if (debug_tcp_bbr)
		cout << "BBR TSO Segs: " << tso_segs << endl;

	return tso_segs;
}


/* Calculate bdp based on min RTT and the estimated bottleneck bandwidth:
 *
 * bdp = ceil(bw * min_rtt * gain)
 *
 * The key factor, gain, controls the amount of queue. While a small gain
 * builds a smaller queue, it becomes more vulnerable to noise in RTT
 * measurements (e.g., delayed ACKs or other ACK compression effects). This
 * noise may cause BBR to under-estimate the rate.
 */
double bbr_bdp(flow_info* fs_ptr, double t_now, double gain)
{
	double bdp, bw, min_rtt, mss;

	if (debug_tcp_bbr)
		std::cout << "BBR calculating BDP..." << endl;

	// To solve the problem of BBR transmitting excessive data
	// unlike its intended behavior, the measured RTTand RTprop
	// are used to reduce the amount of inflight data from each flow.
	// overide!!
	double bbr_gamma = 1.0;

	bw = fs_ptr->BBR_BtlBw;

	min_rtt = (fs_ptr->BBR_RTprop < t_inf) ? fs_ptr->BBR_RTprop : fs_ptr->t_srtt;

	mss = (double)fs_ptr->mss_bytes;

	if (debug_tcp_bbr)
		cout << "Checking the BBR RTT prop: " << fs_ptr->BBR_RTprop << endl;

	if (debug_tcp_bbr)
		cout << "CWND gain: " << gain << endl;
	 
	/* Apply a gain to the given value, remove the BW_SCALE shift, and
	 * round the value up to avoid a negative feedback loop.
	 * bdp = (((w * gain) >> BBR_SCALE) + BW_UNIT - 1) / BW_UNIT;
	 */

	bdp = gain * bbr_gamma * (bw * min_rtt) / mss;

	if (debug_tcp_bbr)
		cout << "BDP (Segs): " << bdp << endl;

	return bdp;
}




// The goal of PROBE_RTT mode is to have BBR flows cooperatively and
// periodically drain the bottleneck queue, to converge to measure the true
// min_rtt(unloaded propagation delay).This allows the flows to keep queues
// small(reducing queuing delay and packet loss) and achieve fairness among
// BBR flows.
//
// see BBRUpdateRTprop() pg 14 BBR RFC and Torvalds linux
//
void bbr_update_min_rtt(flow_info* fs_ptr, seg_info* sh_ptr, snd_info* sv_ptr, double t_now)
{
	if (debug_tcp_bbr)
		cout << "	E. --- Updating the BBR.RTprop Min Filter ---" << endl;

	int in_flight;

	// find the packet rtt, has to be positive in S-XG!!
	double packet_rtt;

	// set prior in flight
	if (fs_ptr->send_high == 0)
		in_flight = fs_ptr->prior_in_flight - sv_ptr->packets_delivered;
	else
		in_flight = fs_ptr->prior_in_flight - sv_ptr->packed_out;

	// set packet rtt
	packet_rtt = t_now - sh_ptr->P_sent_time;

	//fs_ptr->BBR_RTprop = packet_rtt; // check this!!!!!!

	if (debug_tcp_bbr)
		cout << "??? Packet RTT: " << packet_rtt
		<< ", ??? BBR_RTprop: " << fs_ptr->BBR_RTprop
		<< ", ??? BBR RTprop Timestamp: " << fs_ptr->BBR_rtprop_stamp << endl;

	// this bit is like BBRUpdateRTprop() pg 14 BBR RFC and Torvalds linux
	if (t_now > fs_ptr->BBR_rtprop_stamp + bbr_min_rtt_win_sec)
		fs_ptr->BBR_ProbeRTT_round_done = 1;

	// if we have a new min RTT or 10 secs have elapsed
	if ((packet_rtt >= 0) &&
		((packet_rtt <= fs_ptr->BBR_RTprop) || (fs_ptr->BBR_ProbeRTT_round_done > 0)))
	{
		fs_ptr->BBR_RTprop = packet_rtt;
		fs_ptr->BBR_rtprop_stamp = t_now;

		if (debug_tcp_bbr)
			cout << ">>> Updating BBR RTProp: " << fs_ptr->BBR_RTprop << " Secs,"
			<< " RTprop Timestamp: " << fs_ptr->BBR_rtprop_stamp << endl;
	}

	// this bit is like BBRCheckProbeRTT() pg 28 BBR RFC and Torvalds linux
	if ((fs_ptr->BBR_mode != BBR_PROBE_RTT) && (fs_ptr->BBR_ProbeRTT_round_done > 0))
	{
		if (debug_tcp_bbr)
			cout << "BBR Going into Probe RTT... time now: " << t_now << endl;

		// save the lat known RTprop
		fs_ptr->BBR_RTprop_prev = fs_ptr->BBR_RTprop;

		fs_ptr->BBR_mode = BBR_PROBE_RTT;
		bbr_save_cwnd(fs_ptr); // save cwnd (to be done,... done!)
		fs_ptr->BBR_ProbeRTT_done_stamp = 0;
	}

	// this is like BBRHandleProbeRTT() pg 28 BBR RFC and Torvalds linux
	if (fs_ptr->BBR_mode == BBR_PROBE_RTT)
	{
		int probe_rtt_cwnd;

		// find the PROBE_RTT cwnd
		if (bbr_bdp_fraction > 0)
			probe_rtt_cwnd = max((fs_ptr->BBR_prior_cwnd/bbr_bdp_fraction), bbr_cwnd_min_target); // bbr_cwnd_min_target;
		else
			probe_rtt_cwnd = bbr_cwnd_min_target;

		// Maintain min packets in flight for max(200 ms, 1 round).
		if ((fs_ptr->BBR_ProbeRTT_done_stamp == 0) && (in_flight <= probe_rtt_cwnd))
		{
			if (debug_tcp_bbr)
				cout << "PROBE_RTT PIPE: " << in_flight << endl;

			fs_ptr->BBR_ProbeRTT_done_stamp = t_now + (bbr_probe_rtt_mode_ms / 1e3);
			fs_ptr->BBR_ProbeRTT_round_done = 0;
			fs_ptr->BBR_next_round_delivered = fs_ptr->C_delivered;

			if (debug_tcp_bbr)
				cout << "---# PROBE_RTT updating done stamp: " << fs_ptr->BBR_ProbeRTT_done_stamp << endl;

		}
		else if (fs_ptr->BBR_ProbeRTT_done_stamp > 0)
		{
			if (fs_ptr->BBR_round_start > 0)
			{
				if (debug_tcp_bbr)
					cout << "---& ProbeRTT Round Done!!" << endl;

				fs_ptr->BBR_ProbeRTT_round_done = 1;
			}


			if ((fs_ptr->BBR_ProbeRTT_round_done > 0) && (t_now > fs_ptr->BBR_ProbeRTT_done_stamp))
			{
				if (debug_tcp_bbr)
					cout << "---&& EXIT ProbeRTT!!" << endl;

				// unstabe if exit in cyce phase 7! 6 - aggressive, 2 - cautious
				fs_ptr->BBR_GainCycleIdx = 2 + (rand() % 5); // (rand() % CYCLE_LEN);

				// reset
				fs_ptr->BBR_rtprop_stamp = t_now;
				fs_ptr->BBR_ProbeRTT_round_done = 0;

				// BBRRestoreCwnd()
				bbr_set_cwnd_to_recover(fs_ptr, sv_ptr);

				// BBRExitProbeRTT()
				if (fs_ptr->BBR_filled_pipe > 0)
					fs_ptr->BBR_mode = BBR_PROBE_BW;
				else
					fs_ptr->BBR_mode = BBR_STARTUP;

				if (debug_tcp_bbr)
					cout << "CWND: " << fs_ptr->snd_cwnd << ", PIPE: " << in_flight << endl;
			}
		}
	}
}


//
void bbr_update_gains(flow_info* fs_ptr)
{

	if (debug_tcp_bbr)
		cout << "	F. --- Updating the BBR Gains ---" << endl;

	if (debug_tcp_bbr)
		cout << "$ BBR UPDATE GAINS... MODE (0 - STARTUP, 1 - DRAIN, 2 - PROBE BW, 3 - PROBE RTT): " << fs_ptr->BBR_mode << endl;

	switch (fs_ptr->BBR_mode)
	{
	case BBR_STARTUP:
		fs_ptr->BBR_pacing_gain = bbr_high_gain;
		fs_ptr->BBR_cwnd_gain = bbr_high_gain;
		break;
	case BBR_DRAIN:
		fs_ptr->BBR_pacing_gain = bbr_drain_gain;
		fs_ptr->BBR_cwnd_gain = bbr_high_gain;
		break;
	case BBR_PROBE_BW:
		fs_ptr->BBR_pacing_gain = bbr_pacing_gain[fs_ptr->BBR_GainCycleIdx];
		fs_ptr->BBR_cwnd_gain = bbr_cwnd_gain;
		break;
	case BBR_PROBE_RTT:
		fs_ptr->BBR_pacing_gain = 1;
		fs_ptr->BBR_cwnd_gain = 1;
		break;
	default:
		break;
	}

	if (debug_tcp_bbr)
		cout << "Update Gain,... BBR Pacing Gain: " << fs_ptr->BBR_pacing_gain << " "
		<< ",... BBR CWND Gain: " << fs_ptr->BBR_cwnd_gain << endl;

}


// Pace using current bw estimate and a gain factor - BBRSetPacingRateWithGain pg 15
void bbr_set_pacing_rate(flow_info* fs_ptr)
{
	if (debug_tcp_bbr)
		cout << "2. Setting the BBR Pacing Rate" << endl;

	if (fs_ptr->BBR_BtlBw == 0)
	{
		// calc. nominal bandwidth
		double snd_cwnd = (double)max(fs_ptr->snd_cwnd, bbr_cwnd_min_target);

		double mss_bytes = (double)fs_ptr->mss_bytes;

		double rtt_val = (fs_ptr->t_srtt > 0) ? fs_ptr->t_srtt : 0.001; // 1 ms	// 		

		double nominal_bandwidth = ( snd_cwnd * mss_bytes ) / rtt_val; // bits per second 
			
		if ((fs_ptr->BBR_has_seen_rtt < 1) && (fs_ptr->t_srtt > 0))
			fs_ptr->BBR_has_seen_rtt = 1;

		if (debug_tcp_bbr)//(1)//
			std::cout << "snd_cwnd: " << snd_cwnd << ", "
			          << "BBR has seen rtt (0 - NO, 1 - YES)? " << fs_ptr->BBR_has_seen_rtt << ", "
			          << "rtt value: " << rtt_val   << " secs, " 
			          << "t_srtt: "    << fs_ptr->t_srtt << " secs " << endl;

		fs_ptr->BBR_pacing_rate = fs_ptr->BBR_pacing_gain * nominal_bandwidth;

		if (debug_tcp_bbr)//(1)//
			cout << "Nominal Bandwidth: " << nominal_bandwidth << " (Bps) - "
			<< "Setting BBR Pacing Rate... Pacing Gain: " << fs_ptr->BBR_pacing_gain << ", "
			<< "Pacing rate: " << fs_ptr->BBR_pacing_rate << " (Bps)" << endl;
	}
	else
	{
		double rate = fs_ptr->BBR_pacing_gain * fs_ptr->BBR_BtlBw; 

		if (debug_tcp_bbr)
			cout << "BtlwBw: " << fs_ptr->BBR_BtlBw << " (Bps) - "
			<< "Calculating BBR Pacing Rate... Pacing gain: " << fs_ptr->BBR_pacing_gain << ", "
			<< "calculated Rate: " << rate << " (Bps), "
			<< "current pacing rate: " << fs_ptr->BBR_pacing_rate << " (Bps)" << endl;

		if ((fs_ptr->BBR_filled_pipe > 0) || (rate > fs_ptr->BBR_pacing_rate))
			fs_ptr->BBR_pacing_rate = rate;

		if (debug_tcp_bbr)
			cout << "Pipe full? " << fs_ptr->BBR_filled_pipe << " "
			<< "BtlBw-based pacing rate: " << rate << endl;
	}	
}


// Slow-start up toward target cwnd (if bw estimate is growing, or packet loss
// has drawn us down below target), or snap down to target if we're above it.
//
void bbr_set_cwnd(flow_info* fs_ptr, seg_info* sh_ptr, snd_info* sv_ptr, double t_evt)
{
	if (debug_tcp_bbr)
		std::cout << "3. BBR SET CWND" << endl;

	int fid = sh_ptr->flowId, packets_delivered = sv_ptr->packets_delivered;

	// update this later if necessary
	int snd_cwnd = fs_ptr->snd_cwnd, max_packets_out = fs_ptr->max_packets_out, C_delivered = sv_ptr->C_delivered;

	double cwnd_gain = fs_ptr->BBR_cwnd_gain;
		
	int probe_rtt_cwnd, initialCwnd = 1;

	// The BBR BBR.target_cwnd is the upper bound on the volume of data BBR
	// allows in flight. This bound is always in place, and dominates when
	// all other considerations have been satisfied : the flow is not in loss
	// recovery, does not need to probe BBR.RTprop, and has accumulated
	// confidence in its model parameters by receiving enough ACKs to
	// gradually grow the current cwnd to meet the BBR.target_cwnd.

	// BBRUpdateTargetCwnd(), pg 17 bbr rfc 
	int target_cwnd = bbr_target_cwnd(fs_ptr, t_evt, cwnd_gain);

	if (debug_tcp_bbr)
		cout << "Target CWND: " << target_cwnd << ", "
		      << "CWND gain: "  << cwnd_gain   << endl;

	// find the PROBE_RTT cwnd
	if (bbr_bdp_fraction > 0)
		probe_rtt_cwnd = max((fs_ptr->BBR_prior_cwnd / bbr_bdp_fraction), bbr_cwnd_min_target); // bbr_cwnd_min_target;
	else
		probe_rtt_cwnd = bbr_cwnd_min_target;

	// When growing cwnd, don't grow beyond twice what we just probed
	int max_probe_cwnd;

	// debug info
	if (debug_tcp_bbr)
	{
		cout << "->BBR cwnd target: " << target_cwnd << ", "
			<< "Packets delivered: " << packets_delivered << ", "
			<< "Max Pipe: " << max_packets_out << ", "
			<< "C delivered: " << C_delivered << "\n"
			<< "->Current CWND: " << snd_cwnd << ", "
			<< "Packet conservation: " << fs_ptr->BBR_packet_conservation << ", "
			<< "Pipe filled (0 - NO, 1 - YES): " << fs_ptr->BBR_filled_pipe << ", "
			<< "C delivered: " << C_delivered << " segments" << endl;
	}

	/*
	Ref Pg 20 RFC
	If BBR has measured enough samples to achieve confidence that it has filled 
	the pipe (see the description of BBR.filled_pipe in the "Startup" section below), 
	then it increases its cwnd based on the number of packets delivered, while
	bounding its cwnd to be no larger than the BBR.target_cwnd adapted to
	the estimated BDP. Otherwise, if the cwnd is below the target, or
	the sender has marked so little data delivered (less than InitialCwnd) that it 
	does not yet judge its BBR.BtlBw estimate and BBR.target_cwnd as useful, then 
	it increases cwnd without bounding it to be below the target. Finally, BBR 
	imposes a floor of BBRMinPipeCwnd in order to allow pipelining even with small 
	BDPs (see the "Minimum cwnd for Pipelining" section, above).
	*/
	
	// BBRSetCwnd(), see pg 20 bbr rfc
	if (fs_ptr->BBR_filled_pipe > 0)
		snd_cwnd = min(snd_cwnd + packets_delivered, target_cwnd);
	else if ((snd_cwnd < target_cwnd) || (C_delivered < (2 * initialCwnd)))
		snd_cwnd = snd_cwnd + packets_delivered;

	// calc. limit twice what we just probed
	max_probe_cwnd = max(2 * max_packets_out, snd_cwnd);

	// When growing cwnd, don't grow beyond probe limit
	fs_ptr->snd_cwnd = min(snd_cwnd, max_probe_cwnd);

	// Ensure minimum cwnd for Pipelining
	fs_ptr->snd_cwnd = max(snd_cwnd, bbr_cwnd_min_target);

	int cwnd_var_0 = fs_ptr->snd_cwnd_clamp;

	// apply global cap
	snd_cwnd = min(snd_cwnd, cwnd_var_0);

	// BBRModulateCwndForProbeRTT(), see pg 19 bbr rfc
	if (fs_ptr->BBR_mode == BBR_PROBE_RTT)
		fs_ptr->snd_cwnd = min(snd_cwnd, probe_rtt_cwnd); // drain queue, refresh min_rtt

	if (debug_tcp_bbr)//
		cout << "-->BBRSetCwnd()... CWND UPDATED: " << fs_ptr->snd_cwnd << ", "
		     << "BBR BTLBw: " << fs_ptr->BBR_BtlBw << ", "
		     << "BBR RTprop: " << fs_ptr->BBR_RTprop << ", "
		     << "BBR target cwnd: " << target_cwnd << ", "
		     << "BBR Mode: " << fs_ptr->BBR_mode << endl;


}



// Save "last known good" cwnd so we can restore it after losses or PROBE_RTT
//
// BBRSaveCwnd(), pg 19 bbr rfc
void bbr_save_cwnd(flow_info* fs_ptr)
{
	if ((fs_ptr->recover < 1) && (fs_ptr->BBR_mode != BBR_PROBE_RTT))
		fs_ptr->BBR_prior_cwnd = fs_ptr->snd_cwnd;
	else
		fs_ptr->BBR_prior_cwnd = max(fs_ptr->BBR_prior_cwnd, fs_ptr->snd_cwnd);

	if (debug_tcp_bbr)
		cout << "=== bbr_save_cwnd()... PRIOR CWND UPDATED: " << fs_ptr->BBR_prior_cwnd << " ===" << endl;
}



// restore the last-known good cwnd (the latest cwnd unmodulated by loss recovery or ProbeRTT)
//
// BBRRestoreCwnd(), pg 19 bbr rfc
void bbr_set_cwnd_to_recover(flow_info* fs_ptr, snd_info* sv_ptr)
{
	// restore
	if (sv_ptr->recover_flag > 0)
	{
		fs_ptr->snd_cwnd = min(fs_ptr->prior_in_flight + 1, fs_ptr->BBR_prior_cwnd);

		// max packets delivered after recovery
		fs_ptr->max_packets_delivered = fs_ptr->prior_in_flight;
	}
	else
	{
		fs_ptr->snd_cwnd = max(fs_ptr->snd_cwnd, fs_ptr->BBR_prior_cwnd);
	}

	if (debug_tcp_bbr)
		cout << "---> bbr_set_cwnd_to_recover()...\n??? CWND UPDATED: " << fs_ptr->snd_cwnd << ", "
		     << "??? max_packets_delivered: " << fs_ptr->max_packets_delivered << endl;
}

