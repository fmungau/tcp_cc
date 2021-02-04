# tcp_cc
Repository contains code with proposed rule changes for the tcp bbr congestion control algorithm.

Attached are tcp_bbr.cpp and tcp_rate.cpp files with rule change proposals for asymptotic pacing and restoring cwnd to max_packets_delivered as described in the blog article.

https://www.trueux.net/single-post/resolving-the-bandwidth-delay-uncertainty-through-asymptotic-pacing 

The code is purpose built for a multi-core modeller and needs to be modified for testing in other modellers such as NS-3. However, the variables in the code are closely aligned with the BBR v1 Linux implementation with comments referring to the BBR specifications  

The goal of the rule changes is to improve BBR v1 fairness and minimize queue pressure for large RTT ratios e.g., 10X with minimum changes to the BBR v1 source. It is hoped that this conserves all of BBRs current advantages while enhancing performance.
