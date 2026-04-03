Online GNN inference has been widely explored with real-time dynamic graphs. We want to achieve millisecond-level SLOs as a guaranteed result.  
The issues are:
- long tail latency due to imbalanced data-dependent sampling
- large communication overhead for the distributed sampling

The key concepts for this paper are:
1. pre-sampling the dynamic graph in an aevent-driven approach
2. maintaining  a query-aware sample cache to build the complete K-hop sampling results locally for inference requests
# Introduction
Long tail altency due to imabalnced and data depedent sampling use sampling strategies like $\text{TopK}$ and $\text{EdgeWeight}$ require traversing all the neighbors of a vetex toobtain the samples, in this context hte amountof data and necessary computation required by such sampling operations can vary significantly among different seed vertices 
![[Pasted image 20260402151215.png]]
In figure $(b)$ you can see the latency varying with P99 suprassing the average and in $(c)$ you can see data skewness being correlated with number of traversed neighbors, in $(a)$ we can see that sampling is what takes a lot of time and finally in $(d)$ you can see the communication costs skyrocketing when the number of machnes and hops grows. The latter graph is the one explaing the large network comunication overhead, even if results can be cached we have update in dynamic graphs rendere most query caches useless. 
The point of Helios, is to have an **efficient disitrbted dynamic graph sampling** service to tackle SLOs, specically their point is that the pattern for that graph sampling of a given GNN model, i.e. the fan-out number, is determined by how the model is trained. The patterns themslesves will depend on the data distribution itself, if a pattern changes during inference the model performance degrades and becomes unstable, due to shifts in data distribution.

From this observation trying to pre-sample the dynamically updated graphs  and continuously push the complete multi-hop neighborhood samples of each seed vertex to the same machine serving for inference requests. In this approach buildinga complete samplingreuslt for an inference query requires a fixed number of local cache lookups thus sampling latency is reduced.
## Key designs

To get to the statement from before they implement three points:
1. Even-dirven pre-sampling: Helios transfers the sampling computation from real-time inference to the graph update process, speifcally Helos decompoes a K-hop sampling query into K one-hop sampling queries followingthe continuous influx of graph updates.
![[Pasted image 20260403101424.png]]
In this case then we can see what happens better , above you can see a decomposition of a K-hop query, below we have a represenration of how the event driven per-sampling is done
![[Pasted image 20260403113822.png]]
2. Query-aware sample cache: to scale out to the throughut of serving samling query and avoid query lantecy Helios slices target inference verteces into multiple serving ervers and maintains the query-aware sample cache in each srever, Helios trackes dynamically changing one-hop sampling results esentia lfor constructing complete K-hop sampling rsults for each target vertex
These two points are non-trivial to implement and in GNN inference sudden workload bursts are the norm, this scalability is a challenge.
3. Sampling/Serving sepration, Helios must adress also the latter point, we use a decupled archicture of workers pre-sampling the dynamic graph and push the results to the serving workers. Each serving worker maintains a query-aware sample cache and generates sampling results for inferene requests without uncurring in communication overhead. These two sets for workers can scale independently so that Helios can both high concurrency inference requests and hgh throughput graph updates. The separation architecture makes it challegenging to trace the dynamically changing K-hop sampling results for the sampe cache and we address this with an efficient event-driven subscription mechanism.
Through empirical validtion Helios achieves a P99 latency within 50ms, each worker serves 4000 queries/s while exhibiting linear scalability, the pre-sampling throughput if a single sampling worker exceeds 1.49M record/s and demonstrates again near-linear scalability.

Compared to to other Sotaswe have x67 serving increase and x32 latency reduction.

The key points then are:
- Helios, a system that shifts the ad-oc dynamic graph sampling into a graph update process through event-driven pre-sampling
- Query aware sample caching, Helios uses this achieve latency SLOs with efficient local cache lookups
- Develop a separated sampling/servin distributed architecture allowing Helios to scale almost linearly for both graph update ingestion and GNN inference serving


Given the classical formulation for a GNN
$$
a_v^l=Aggregate(h_u^{l-1}|u \in N(v))
$$
$$
h_v^l=Update^l(a_v^l,h_v^{l-1})
$$
now in this context a simple traiing will with these steps:
- select a minibatch of training vertices
- uniformly sampling multihop neighbors base oVn specified fan-outs
	- Note that sampling the graph like this might introduce gradient bias especiall wih small eighborhoods, you can actually mitigate it using techniques like reusing historical activations nad dropout
- collecting features of training vertices and their sampled neighbors
- apply $Aggregate$ and $Update$ functions for forward, then do a backward prop
Such a model is then deployed of course, beow you cn observe th typical dployment/training workflow for dynamical graphs
![[Pasted image 20260403114545.png]]

Of course training on a GNN ofr large scale graps is resource-intesive and time-consuming, in industrial applicatons GNN are updat at intervals of hours, days or even longer with an ofline approach, incremental training can reudce the traiing costsby updating the model using the newly arriving data or update in the graph rater than retraining the entire model from scratch.

The incremental training fails however to also caputre realtime updates at the millisecond level happening in the real dynamic graphs, inference can doe this instead and use hte last update to update the representations, this process of online inference involves sampling multi-hop neighbors on dyamic graphs for the vertex idicated in the request and employing the GN models to infer the vertex representation.

In this context we havethe afore mentioned long tail latency and large networ communication overheads from distributed multi-hop sampling.
## Testing latency & communication overhead
First we can invistiage the long tailed latency issue, in the preivous figure we have shown the P99 latency of both systems bieng way higher then their average latency, the computtaion cost required by the ad-hoc samplng query varies baed on difrent sampling seed due to the skewness of real-world graphs
- for instand in a **timestamp based top-k sampling** to select the k neighbors with the largest timestamps for a vertex $V_i$ the timestamp of every edge connection it to its neighbors has to be collected and sorted. Supernodes exists in real world graphs, i.e. with a lof of neigbors, and traversing such neighborhood of supernodes increase I/O and computation intensity.
First efeccts of skewness through TigerGraph and verying fanout in the range $[25,10]$ they randomly selecte 200,000 of the two.hop timestamp-based sampling. There is a difference of 100x in the accessed number of neighbors vertices and query latency can surge up to 20x on averge.

As fore the network communication overhead, the scale of graphs and the workload of concurrent inferece requests in industrial settings of seceed the capcity of a single machine, a distributed graph DB used usually to store graph data and execute sampling qurues in a distributed cluster where each machine sotres a partition of the graph given that GNN inference typically involves multi-hop sampling in a distributed storage of course thte communication overhad will happen during neighbors traversal and fetchin the features of neighbors. 

Te use dataset is **Interactive**. 

They tested the number of sampling hops and lcuster size on distributed sampling latency, going from 2 to 3 increases of 65x the latency of sampling. each addtonal hop requires an addiotnla cross machine comunication round which is very costly.
