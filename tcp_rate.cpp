//============================================================================
// Name        : SureLinkXG_tcp_rate.cpp
// Author      : Frank M
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include "threadprocessor.h"

alignas(64) int debug_tcp_rate = 0;


// Snapshot the current delivery information in the skb, to generate
// a rate sample later when the skb is (s)acked in tcp_rate_skb_delivered().
void  tcp_rate_seg_sent(bbr_snd* bbr_sender, seg_info* sh_ptr, double t_now)
{
	if (debug_tcp_rate)
		cout << "~~~snapshot the current rate deilvery info in the seg, t_now: " << t_now << " sec" << endl;

	int fid = sh_ptr->flowId;

	//cout << "?? whats my PIPE: " << fs_ptr->Pipe_1 << endl;

	sh_ptr->P_delivered      = bbr_sender->C_delivered; // count segs... not bytes 
	sh_ptr->P_is_app_limited = 0;

	// In the linux code, the tcp_socket tp translates to C
	if ( (bbr_sender->inflight == 0) || (bbr_sender->init_rto == 1) ) 
	{
		bbr_sender->C_first_sent_time = t_now;
		bbr_sender->C_delivered_time  = t_now;

		if (debug_tcp_rate)//(1)//
			cout << "--> Initializing rate sample connection times to: " << t_now << " sec" << endl;

		// done init
		bbr_sender->init_rto = 0;
	}


	// in the linux code, TCP_SKB_CB(skb) translates to P
	sh_ptr->P_delivered_time  = bbr_sender->C_delivered_time;
	sh_ptr->P_first_sent_time = bbr_sender->C_first_sent_time;
	
	// The time when the packet was sent.
	sh_ptr->P_sent_time = t_now;

	if (debug_tcp_rate)
		cout << "P_delivered: " << sh_ptr->P_delivered << " Segs, "
		     << "sh_ptr->P_first_sent_time: " << sh_ptr->P_first_sent_time << " Secs, "
		     << "Time sent: " << sh_ptr->P_sent_time << " Secs" << endl;

	if (debug_tcp_rate)
		cout << "~~~ Done with Rate Sample Info!! ~~~" << endl;
}


// When an skb is sacked or acked, we fill in the rate sample with the (prior)
// delivery information when the skb was last transmitted.
void tcp_rate_seg_delivered(flow_info* fs_ptr, seg_info* sh_ptr, double t_now)
{
	if (debug_tcp_rate)
		cout << "1. Rate sampling - segment delivered..." << endl;

	if (debug_tcp_rate)
		cout << "===>>> Executing ACK arrival half of the BBR Algorithm... fill rate sample" << endl;

	int fid = sh_ptr->flowId;

	if (sh_ptr->ack_seq > 2)
		fs_ptr->C_delivered += fs_ptr->packets_delivered; // segs counter
	
	// update the delivery time
	fs_ptr->C_delivered_time = t_now;

	if (debug_tcp_rate)//(0)//
		cout << "C_delivered: " << fs_ptr->C_delivered << " Pkts, "
		     << "P_delivered: " << sh_ptr->P_delivered << " Pkts, "
		     << "RS Prior delivered: " << fs_ptr->rs_prior_delivered << " Pkts" << endl;

	// update rate sampling info - bytes 
	if (sh_ptr->P_delivered > fs_ptr->rs_prior_delivered) 
	{
		if (debug_tcp_rate)
			cout << "   A. --- updating the rate samples times & quantities ---    " << endl;

		// mark the end of receiving a round
		fs_ptr->rs_prior_delivered = sh_ptr->P_delivered;
		fs_ptr->rs_prior_time = sh_ptr->P_delivered_time;

		// Find the duration of the "send phase" of this window:
		// Record both segment send and ack receive intervals
		fs_ptr->rs_send_elapsed = sh_ptr->P_sent_time - sh_ptr->P_first_sent_time;
		fs_ptr->rs_ack_elapsed = fs_ptr->C_delivered_time - sh_ptr->P_delivered_time;

		// Record send time of most recently ACKed packet:
		fs_ptr->C_first_sent_time = sh_ptr->P_sent_time;

		if (debug_tcp_rate)
			std::cout << "->fs_ptr->rs_send_elapsed: " << fs_ptr->rs_send_elapsed << ", "
            		  << "fs_ptr->rs_ack_elapsed: "  << fs_ptr->rs_ack_elapsed << ",\n"
			          << "->fs_ptr->C_first_sent_time: " << fs_ptr->C_first_sent_time  << ", "
			          << "RS Prior delivered: "          << fs_ptr->rs_prior_delivered << " Bytes, " << endl;
	}

	if (debug_tcp_rate)
		cout << "RS Prior time: " << fs_ptr->rs_prior_time << endl;
}


// Update the connection delivery information and generate a rate sample.
void tcp_rate_gen(flow_info* fs_ptr)
{
	if (debug_tcp_rate)
		cout << "2. Rate sampling - determining the TCP connection data delivery rate..." << endl;

	int fid;

	// get fid 
	fid = fs_ptr->fid;

	// check that something was delivered
	if (fs_ptr->rs_prior_time == 0)
	{
		if (debug_tcp_rate)
			cout << "---> exiting process... the P.delivered_time from the most recent packet delivered is zero." << endl;

		return;
	}
	
	if (debug_tcp_rate)
		cout << "   B. --- updating connection rate delivery info ---    " << endl;

	// Use the longer of the send_elapsed and ack_elapsed
	fs_ptr->rs_interval = max(fs_ptr->rs_send_elapsed, fs_ptr->rs_ack_elapsed);

	if (debug_tcp_rate)//(0)//
		cout << "->rs interval: " << fs_ptr->rs_interval << " secs\n"
		<< "->rs_send_elapsed: " << fs_ptr->rs_send_elapsed << " secs" << ", "
		<< "fs_ptr->rs_ack_elapsed: " << fs_ptr->rs_ack_elapsed << " secs" << endl;

	// key to our bbr rate calculation
	fs_ptr->rs_delivered = (fs_ptr->C_delivered - fs_ptr->rs_prior_delivered);

	if (debug_tcp_rate)//(0)//
		cout << "->rs_delivered: " << fs_ptr->rs_delivered << " Segs\n"
		     << "->fs_ptr->C_delivered: " << fs_ptr->C_delivered << " Segs, " 
		     << "fs_ptr->rs_prior_delivered: " << fs_ptr->rs_prior_delivered << " Segs" << endl;

	// Normally we expect interval_us >= min_rtt. Note that rate may still be over-estimated when a spuriously
	// retransmistted skb was first (s)acked because "interval_us" is under-estimated (up to an RTT). However continuously
	// measuring the delivery rate during loss recovery is crucial for connections suffer heavy or prolonged losses.
	//if (fs_ptr->rs_interval < fs_ptr->BBR_RTprop)
	//{
	//	fs_ptr->rs_interval = -1;
	//	return;		
	//}
	
	// debug...
	if (debug_tcp_rate)
		cout << "--- Recalculating Delivery Rate at Bottleneck ---" << endl;

	double damping_factor, measured_BtlBw, estimated_Btl_Bw, delta_BtlBw;

	estimated_Btl_Bw = fs_ptr->BBR_BtlBw;

	// rate sampl available?
	if ((fs_ptr->rs_interval > 0) && (fs_ptr->rs_delivered > 0))
		measured_BtlBw = ((double)fs_ptr->mss_bytes
			* (double)fs_ptr->rs_delivered) / fs_ptr->rs_interval;
	else
		measured_BtlBw = 0;

	damping_factor = (100 - bbr_pacing_margin_percent) / 100;

	delta_BtlBw = (measured_BtlBw - estimated_Btl_Bw);

	fs_ptr->rs_delivery_rate = damping_factor * (estimated_Btl_Bw + delta_BtlBw); //damping_factor * measured_BtlBw;

	if (debug_tcp_rate)//(f_id == 0)
		cout << "Measured delivery rate: " << measured_BtlBw << " Bytes per second,\n"
		<< "Rate sample deilivery rate: " << fs_ptr->rs_delivery_rate << " Bytes per second" << endl;

	if (fs_ptr->rs_interval < Min_RTT)
	{
		if (debug_tcp_rate)//
			cout << "RTT est too low!! Flow - " << fid << " "
			<< "RTT: " << fs_ptr->rs_interval << endl;
	}
	
}

