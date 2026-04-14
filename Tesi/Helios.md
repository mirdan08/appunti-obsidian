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


## System design 

For all the problems with sampling and caching Helios is proposed, a specialized non-graph-DB dynamic graphi sampling service designed to fulfill SLOs on online GNN inference. Hrlios shifts the ad hoc sampling query exection into the graph update process, using its sampling/serving separation approach the sampling workers constantly refresh sample results by an event driven è pre-sampling mechanism and push the multi-hop presampled results to a designated serving worker ensuring GNN infernce queries to be completed within a fixed number of local cache lookups in a single serving worker. 
![[Pasted image 20260403155737.png]]


This is Helios's whole system architecture ot deploys $M$ sampling workers and $N$ serving workers. 
- Graph updates are partitioned into $M$ partition for each one a sampling worker is assigned.
- Inference reuests are partioned into $N$ partitions using the vertex seed id and each worker handling on partition.
For this they used Kafka, to store ad transfer input of sampling and serving workers. Helios allows users to configure their sampling query and the coordinator:
- register the user-specified query 
- decomposes it into one-hop queries the K-hop queries 
- initializes the one-hop queries on the sampling and serving workers.

The coordinator has these  models the data dependency between one-hop queries as a directed acyclic graph and sends it to all workers to facilitate the management of the subscription table and to construct complete sampling results. While the service is running the coordinator monitors the liverness of all workers via heartbeats and periodically triggering checkpointing for fault tollerance. The separation of sampling and serving processes enables independent scaling for both sampling and scaling workers, the system can thus adapt to dynamically changing workloads ensuring stable serving latncy during themporary graph update bursts in the sampling workers.
Note: the authors recommend first assessing the thorughput of a single sampling/serving worker then servig workers according to the aplication's requirements like graph update rates and inference query rates.
![[Pasted image 20260407102045.png]]
### Sampling worker
This is the  graph for sampling and serving worrkers in detail, gfraph updates arrive continuosly the sampliong worker updates for each update the results of the one-hop sampling queries used to contruct later on K-hop sampling reuslts for querying a vertex. 
### Graph updates
Helios has two kind of updates:
- vertex update called $\text{VertexUpdate}(V_i)$: it's the addition of a new vertex or a previous one feature update
- edge update called $\text{EdgeUpdate}(E_i)$ :it's the addition of a new edge
Note: it's an APPEND ONLY dynamic graph approach, common for manmy logging-based graph systems.
### Partioning strategies
With $M$ sampling workers graph updates are sliced evenly into $M$ partitions on assgined exclesively to a worker. A predefined hash functions determines the partitions IDs of vertex updates:
- for an edge update $E_i$ if the graph is undirected $E_i$ is replicated in the partitions of both its source and destination vertices, when directed you must configure two policies
	- $\text{BySrc}$ stores the edge in the source
	- $\text{ByDest}$ determines 
	- $\text{Both}$ will beahave as for indirected case
#### Sampling workers components
each wroker is made of three parts:
1. A **reservoir table** for each one-hop query, the $key$ is the vertex ID and values are the sampled neighbor vertex IDs for the corresponding vertex, the capacity is determined by the query fan-out
2. A **feature table** storing dynamically updated features for each vertex in the lòocal partiton of this sampling worker
3. A **subscription table** for each one-hop query which maintains the list of serving workers subscribing to the feature and sample updates of key vertices in the reservoir table of this query.
#### The execution engine for graph update ingestion
IO and compuattions insampling workers are pipelined, interference among different types of workloads is minimized by isolating them into distinct threads implemented thorugh an ecotr-based framework. 
Threads are specialized into three types:
1. Polling threads continuously fetch the latest graph updates from the input queue
2. Sampling threads execute the one-hop query and update subcription tables
3. Publisher threads push the sampled results to the corresponding Kafka queues according to the subscription table
Workloads are prioritized by assigning them to a larger thread pool by Helios.

### Serving workers
In the same figure as before we can analyze the serving workers structure.
- The serving owrker subscribes to the outèputs of the sampling workers and promptly responds to the sampling queries in inference requests. 
- Inference time is minmal withoput network overheads and the workers can scale linearly with more workers.
- The number of cache lookups required to generate the full sampling result is bounded by the product of the *multi-hop sampling fan-outs*.

Tail latency is reduced even when sampling on highly skewed graphs.

#### Serving workers components
Each serving worker maintains query-aware sample cache that consists in two parts:
1. a sample table for each one-hop query that stores the pre-sampled neighbors of vertices
2. a feature table that maintains the latest feature for all the existing vertices in the sample tables including the seed and sampled neighbor vertices
The TTL threshold removes the strale data in the sample cache.
#### Execution engine of sampling query serving
The front-end nodes routes inference requests to serving servings according to the IDs of their seed vertices, on receiving a sampling query for an inference request the serving worker constructs the query result by referencing its local sample cache. 

Serving workers are also designated to handle different workloads to different phgysical threads byt the distributed actor-based framework:
1. Polling threads continuously fetch the latest samples from the input queue.
2. Data updating threads update the sample and feature table
3. Serving threads execute the sampling queries received  from the front-end nodes and further send sampled results to model services like TF-serving

## Event driven pre-sampling
the pre-sampling is triggered by the arrival of graph updates, the pre-sampling K-hop query mainly consists of three steps:
1. K-hop query decomposition
2. event-driven reservoir sampling
3. updating subcription table and publishing samples
### Query decomposition
Pre-sampling a multihop query requires the coordinator to decompose a K-hop sampling query into K one-hop sampling queries, we have already seen this example.
## Even driven reservoir sampling
When the decomposed queries are received the graph updates from the input queues are retrieved from the input workers, from their input queues, and that refresh their sampling results for each one-hop query.
A reservoir table is maintained for each one-hop query the keys in each reservoir table are the target vertex Ids of the corresponding one-hop query. The capacity of such cells is determined by the fan-out of the query.

The features table in a worker holds the latest features in its local partition, each sampling worker handles a partition of the graph updates ensuring no duplication among all sampling workers for the keys in their reservoir/features tables.

When an edge $E_k : V_i \rightarrow V_j$  is updated we have a few scenarios:
- if $V_i$ is the target vertex of a one-hop query $Q_k$ the sampling results for $V_i$ under $Q_k$ are updated based on $Q_k$'s strategy,
- if $V_j$ is chosen as a new sampled for $V_i$ then $Q_k$'s reservoir table is updated.
- if $V_i$'s cell value is full a previously sampled neighbor of $V_i$ is replaced by $V_j$, otherwise is a newly added fo $V_i$ 
Upon receiving a vertex update the sampling worker updates the corresponding entry in the fgeature table with the latest vertex feature.

Helios supports sampling strategies like $\text{Random}$ and $\text{TopK}$ the data distribution of reservoir sampling is the same as ad-hoc sampling. 
When determing if a neighbor $V_j$ of vertex $V_i$ sholden be chosen as a new sample:
- the $\text{Random}$ strategy generates a random number $p$ in range $[1,x]$ where $x$ is the total number of input ewdge updates to the cell value of $V_i$ at this moment. if $p$ is less than the cell capacity $C$ the $p$-th item in the cell is replaced by $V_j$.
- The $\text{TopK}$ strategy is timestamp-based and compares the timestamps of the incoming graph update and the holdest cell value are replaced by the incoming new ones.
### Subscription update and sample publish
![[Pasted image 20260407122823.png]]

In this figure we can see the subscription tables being handled with an example from the previous 2-hop query.

the subcription table trackes how sample results are disseminated to serving workers, after pre-sampling with a graph update.

THe sampling workers distributed sample updates to the serving workers based on their subscription tables. Note that when vertices are no longer under the subscription ofg specific serving worker tha sampling workers also enqueue an update message, when the serving workers retrieves the message from the the designated sampled queues they also update caches according.

## Discussion
Helios is designed for applications with stringent latency SLOs, tradeoffs are adopt w.r.t. sampling costs with performance of real-time serving. This will lead to some sampling results never accessed but such cases are limited since:
1. inference reque4sts continuously arrive and the sampled results are accessed not only when they are used as the sampling seeds but also as the multi-hop neighbors of some vertices.
2. sampling results of the vertices that are never used are updated less frequently.
This is also validated later on in the paper experimentally.

## Query aware samples caching

As we know each serving workers has a query-aware samples cache, holding the complete K-hop neighbors and the corresponding features.
![[Pasted image 20260407150150.png]]
In the figure above we can see the KV-stores using the hybrid memory disk of RocksDB.

#### Serving sampling queries 
In Helios when an inference request is recevied the request is rotued to a specific serving worker according to the vertex ID.
After obtaining the IDs of sampled vertices the serving worker then accesses the faeture tablew to retrieve the features of all sampled vertics to compose the final sampling results. Assuming that the fan-outs of the K-hop $C_1,\dots,C_K$ :
- the number of lookups operations in the sample table can be calculated as $\Pi_{i=1}^{K-1}C_i$    
- the number of lookups operations in the feature table is calculated as $\Pi_{i=1}^{K}C_i$ 

The query aware samples cahce optimizes the sampling latency and facilitates linear scaling of serving thorughput , by eliminating network communications. Constrained by consistent cache lookups costs Helios manage.
#### Cache Update
Each serving worker continuously polls the sample queues to get the update sample data which are used to update the sample and feature table. Helios only caches the sampled graph topology and features data in serving workers which are much smaller compared to the original graph.. 
When scaling out serving workers the cache size of single worker decreases with more serving workers deployed even though the caches could partially overlap among different serving workers.
![[Pasted image 20260407185827.png]]
in the figure above we can see the serving workers cache size going lower and lower with the number of sering machines in real world GNN inference workloads the graph updates occur at intervals of several seconds or even longer granularity Helios achieves second-leve ingestion latency under input rates of millions of updates per second which is sufficient to reflect the latest graph updates in online GNN inference scenarios.

Different consistenecy guarantees are evalòuted between graph updates assuming that there are new graph  update and concurrent inference requests,  for sampling queries on dynamic grapohs a single graph update can result in multiple neighbor updates to K-hop sampling results for relevant vertices, we study different cases
1. Strong consistency guarantee when graph update are completely ingested query serving can observe 100% updates which is an optimal case where all the latest graph update are immediately visible to serving requests
2. When graph updates are not completely integestedthe query serving can ovbserve 0% of the new updates
3. Under eventual consistency when gvraph updates are completed query serving can observe all updates like case 1 
4. when graph updates are not completed query serving can partially observe updates e.g. 50%
Since a GNN can aggregate multi-hop neighbor information eventual consistency allows Helios to observe many sampling updates as possible during query serving enabling Helios. Helios can reflect graph update more rapidly, and grants eventual consistency.