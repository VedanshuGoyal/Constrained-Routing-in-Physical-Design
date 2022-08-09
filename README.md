# constrained-routing-in-physical-design

This is algorithmic challenging problem in which we have a find a routes between pin groups of a single bus.

We have implemented an effective and efficient topology matching bus routing algorithm considering obstacles and non-uniform track configurations.
• A DAG-based bus routing algorithm has been proposed to efficiently connect all bits within a bus in the specified topology.
• We have implemented our algorithm for the single bus, to use it for the multi-bus we have to implement LCS bus-clustering method to find the order of buses, and then proceed one by one with our 
single bus algorithm
