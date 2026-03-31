Real world graphs are dynamic with frequent updates to structures and features. for GNNs this is very problematic, existing *vertex-wise* and *layer-wise* approaches are redundant, with large neighborhood travesarls and with high communication costs especially in the distributed setting. Here approximated neighborhoods by sampling are not considered we want deterministic embeddings, which can limit low latency inference.
RIPPLE++ tries to address this by creating a model that incrementally propagates into affected neighborhoods using GNNs semantics. It accomodoates all common graph updates like addition/deletion of edge\ and addition/deletion/update of vertexes. Both on single machines and distributed deployments.
# GNNs inference on static to dynamical graphs
GNNs are very expressive when working on large-scale graphs with million to billions of vertices and edges.  In this case inference for a single node, say $D$ in the image below, requires calculated the L-hop neighborhood of the target vertex, called the "computatioal graph", to generate L-layer embeddings which are what we care about. 
![[Pasted image 20260331092157.png]]
In an online setting the simple approach is to perform forward pass once on all the graph entities and cache the preicted results to return them in a simple lookup.

Note that in aggregating neighborhoods usually at inference time we can sample only a part of the neighborhoods but this leads to approximate and non-deterministic results, not our case.
![[Pasted image 20260331094049.png]]
In static graph there two methods:
- vertex-wise:just do the normal forward pass, just aggregate the L-hops neighborhoods
- layer-wise: is employed for bulk processing we compute the embedding for each hop on all vertices of the graph on a layer and used to calculate the next embeddings of all vertex at the next layer.

![[Pasted image 20260331093346.png]]
In the vertex-wise we risk L-hop neighborhoods explosion depending on the vertex L-hop neighborhood whihch might surge memory usage and computational demands.
In layer wise instead we can control much better the computational graph explosion by using the overlapped vertices in the same layer with the neighborhoods of proximate vertices and avoid redundant computation.

Graphs might evolve continuously in presence of vertex/edge additions/deletion/ features update, in a dynamic graph this occurs on per second basis changing the GNN output.
Layer-wise is preferred to control the computational graph size, but for latency sensitive applications its too time consuming, we can limit to recomputation of the vertex labels only for those falling within L-hop out-neighborhood, the *affected neighborhood* on entities that have been updated in the graph, this cascading efects grows with the average degree of the graph and number of updates applied in each batch as more entities fall within affect neighborhood of the modified vertices.
![[Pasted image 20260331094338.png]]

In a Layer-wise approach the whole graph and its embeddings must be present in memory, this is not feasiable not even with 128gb of RAM, managing this in a single-machine process is very difficult indeed so a distributed execution of the incremental computation is needed.

# RIPPLE++
RIPPLE++ proposes a unique approach that applies deltas to undo previous embeddings aggregations and redo them with updated embeddings to reduce sharply recomputation in both single-machine and distributed settings. For a subset of the neigborhood $V' \subseteq V$ we have that $k'=|V'| \lt k=|V|$ usually an order of magnitude smaller. 
The framework supports linear,non-linear and attention-based aggregation with deterministic and accurate behaviour with a distributed approach.
The contributions are formalized as:
1. A scalable frmework for incremental GNN inference support streaming edge and vertex additions, deletions and feature updates for GNN models including monotonic,accumulative aggregators and attention-based architectures.
2. Detailed comparsion agains the full computation , noted as $\textbf{RC}$ characterizing the best conditions for such model
3. Design a locally aware routing strategy to place new vertices minimizing edgecuts and improving distributex execution under graph evolution compared to hash based routing.
4. Perform detailed experiments on **RIPPLE++**, compared against another SOTA Inkstream and other baselines.
Note that here 
- monotonic aggregations like $\min,\max$ and graph attention mechanisms  like $\text{GAT}$ are an extension not present in other SOTAs.
- additions and deletions of vertex/edges are a new
- the comparison is done against full recompute
- the locality aware algorithm provides a baseline against edge cuts and improves load balancing in a distributed setting, beyond hash based routing
- expand experiments to two new incremental GNNs and study these aspects
	- effect of increasing GNN layers
	- scaling to 20M updates the evaluation
	- comparison against InkStream
For short this is the inference rule of a $\text{L-layer GNN}$ where for $l=1,\dots,L$ we have
$$
x_u^l=\text{Aggregate}^l(\{h_v^{l-1}|v \in N(u)\})\\
$$
$$
h_u^l=\sigma(\text{Update}^l(h_u^{l-1},x_u^l))
$$
in the first one we have $\text{Aggregate}$ function aggregating the embedding vetors of $u$'s neighbors from layer $(l-1)$ , the result is then used in the $\text{Update}$ function which contains the **learnable parameters of our network**, passed then through a non linear function $\sigma$ . This happens $L$-times untile we reach the final layer $h^L$ for the training vertex and thats it for the forward pass.
The backward pass just goes backward across the L layers to update the parameters using the loss function.
At inference time only the forward pass is performed to get the final layer embedding which maps to the predicted label. In a static graph this can be precalculated to avoid compute at inference time.
## Inference on streaming graphs
pre-calculating the final layer embeddings and labels for all vertices is not beneficial for dynamic graphs, for each update a cascade of embedding updates happens at each hop changing the embeddings and thus the labels of all vertices at the final hop.
![[Pasted image 20260331112154.png]]
Here we see what happens with a sum aggregater and a 2-layer GNN. adding a new edge provokes a lot of embeddings changes due to the neighborhood of $A$ changing requiring the changing of its embeddings $h_A^1$ and $h_A^2$ at all levels, this provokes the neighboring embeddings of $\{B,D\}$ to change at the next layer, so $h^2$, thus a single change affected $\{A,B,D\}$. This demonstrates that not all layers change each time an we don't have to update all vertices for each fine-grained change. But the fraction affected vertex might go from <10% to >30% easily depending on the dataset and GNN architecture.

In streaming graphs requests are distinguished into two kinds:
- Trigger-based: at each change to the specified target vertex the application is notified.
- Request-based: we use a pull-based model where th application queries the label of a specific entity.
We have different requirements for these two cases e.g. update propagation can be done lazily for rarely accessed vertices in the request based model. Here is treated the trigger-based inference model.
## Aggregation functions

in existing reserach linear accumulatiove functions like $\text{sum},\text{mean},\text{weighted sum}$ are used, sometimes we also consider non-linear monotonic functions like $\min,\max$ and in recent works we also have  graph attention called $\text{GAT}$  .
Some already proposed models use them:
- GraphConvolutionalNetworks use weighted summation
- GraphSAGE uses sum or mean
- GraphAttentionNetworks us an attention based summation
- GraphIsomorphismNetworks showcase sum to display it's expressive power
- Other works display the GNN modules expressivity by using a combination of aggregators like sum and mean or linear an non-linear aggregators.
All these results are encompassed and analyzed using all the aggregations functions in the table below

![[Pasted image 20260331114218.png]]

# System Design
RIPPLE++ supports trigger-based applications that need to be notified of changes to predictions of any graph entity upon receiving updates, as soon as possible. It uses incremental computation over a batch of updates with a smart delta of prior embeddings to avoid redundant computations. It works in both  single-machine and distributed execution settings to accomodate larger and smaller sizes of graphs.

The system starts from iitially calculate embeddings for all graph entities prior to new updates streaming in, specifically:
- for each vertex $u \in V$ its root level features $h_u^0$ and intermediate features $h_u^l.l=1,\dots,L$  
- we have the initial embeddings matrices $H_{T_0}^l$ at layer $l$ are generated using the model itslef and the current labels of all vertices can be extracted from the last layer embeddings i.e. $H_{T_0}^L$ and servesd as a starting point.

The system operates in two ways completely incremental and hybrid incremental.

The system can be easily extend to other vertex ort edge based tasks.

The supported operations are:
- adding new vertex
- removing a  vertex
- adding new edge
- removing an edge
- update a vertex's embbeding on the root level
Note: sometimes adding an edge contains a new vertex thus you also perform the corresponding operation.

The updates arriva contiuosly and are batches into fixed batch sizes $bs$ that are applied to the graph triggering the comptuation on affected entites. The rates are 100-1000 updates per second which in bulk should reduce overheads and redundant computations  so higher throughput is expected. In this case the fixed batch size is treated as a hyperparameter.

## The baseline

First we have a baseline the performs layer-wise recomputing called $\text{RC}$ scoped to teh neighborhood of updates. and when a root layer change happens we have the usual cascading updates starting from the root vertex and all the embedding in the L-hop neighborhood.
![[Pasted image 20260331115918.png]]


In the image we see it bery well, adding a simple edge $(C,A)$ entails a cascade of recomputations in $\text{RC}$ m in particular if you look at $(b)$ you acn see that the cascade of neighborhood there are a lot of fetching done on the distributed setting.

In this case the various operations behave differently
- Edge addition alters aggregated embedding of the target vertex and as we have seen causing the cascade of computations.
- Edge delition will affect the embeddings as the addition does, the same computation happens as before with one less neighbor.
- Vertex addition does not trigger any updates aside from its own embeddings, it doesn't any edge connection so no other nodes aside from itself are affectd. 
	- Vertices can be created as a part of an edge addition, thus if $(u,v)$ is added and the sink exists then we trigger  again all the cases from before if instead the sink does not exist then there no edges to hop to and only the sink itself updates its own embeddings, no cascade of updates.
- Vertex delition makes all edges connected to it disappear all the deleted out neighborhood is affected up to L hops, the neighborhoods might be very large so this can become the most expensive operation.
- Vertex feature update affters the out-neighborhood similarly to vertex delition, it impacts all out neighbors of $u$ when $h_u^0$ changes creating a casce of $L$-hops.
All this operations demonstrate that layer-wise recomputation requres taht to update the $h_v^l$ embedding requires all the previous layer $h_u^{l-1}$ of all its neighbors in $u$ even if not all of them changed. This fact alone entails wasted computation since we need all in-neighbors embeddings to get the new embeddings of the sink  vertex and also this grows exponentially downsttream.
## Incremental update propagation

RIPPLE++ works similarly to $\text{RC}$  but it $\text{Applies}$ the graph updates onto the current graph and identifies root vertices of these mutations, this is a BFS traversal from *each* root vertex of an update to get downstream vertices affected at each hop and whose embeddings need to be incrementally updated this forms the $L$-hop propagation tree.

We perform an iterative vertex-centric computation within the propagation tree, with vertices at distance $l$ fro mthe root vertex participating in hop $l$, we can see this as a BSP model with $L$ superstep one for each hop/layer of the model. This is a look-forward model propagating updates using a message-èassing with barrier synchornization between hops.
Within the $l$-th superstep we perform these computations:
- $\text{Compute}$: computer the update embddings $h^l$ based on the current ones $h^{l-}$  and the incoming message from the previous up for $l>0$ 
- $\text{Prepare}$:prepare the outgoing messages to be sent to its out-neighbors
- $\text{Send}$:send the messages to the sink vertices, the message cartries incremental updates rather than the raw embeddings.
Each vertex maintains L logical *inboxes*, one for each hop at which it may be present from a vertex being updated. Affected neighbors at hop $(l-1)$ of the propagation tree will place their incremental message ot his vertex in its hop-$l$ , the inbox will $\text{Aggregate}$ all the messages from the in neighbors and maintain only one aggregate message at the beginning of the next hop.
![[Pasted image 20260331124629.png]]

This image indicates all these phases, Apply is just the boostrap for when a change happens, we will see later on how the updates are transmitted. 

Note that all the inboxes at hop $l$ will be processed by vertices after all vertices at hop $(l-1)$ completed their compute and send operations consistent with the BSP superstep barrier alternating communication and computation phases.

This is a push based approach as vertices are affected only by the sent messages in their inbox so each vertex doesn't check all its in-neighbors like in a layer-wise approach.
## Incremental update computation

An incermental message is sent from a source vertex at hop $l$ to its sink vertices at hop $(l+1)$ an is meant to nullify the old stale embedding $h^{l-}$  and include its contribution of its current embeddings $h^l$ on the sink vertices creating a *delta*. 

## Linear aggregators
We now go on and understand the propagation of updates in RIPPLE++ for the summation aggreation function:
- $u$ embedding is updated from $h_u^{l-}$ to $h_u^l$ :its a direct update to the feature vertex of $u$, meaning that $h_u^0$ changes or that $u$ falls at hop $l<L$ of the propagation tree for another edge/vertex update.
	- if $u$ is not at the leaf of the tree it will send a message $m_{ub}^{l+1}$ to the $(l+1)$ inbox of its sink vertex $v$ 
	The contentets of $m_{ub}^{l+1}$ are made s.t. they invalidate $h_u^{l-}$ on $h_v^{l+1}$ and include the contribution of the new embedding $h_u^l$ e.g. for a sum operator $m_{uv}^{l+1}=h_u^l-h_u^{l-}$ , to see the results look at the image below unrolling a message passing with such dynamics
	![[Pasted image 20260331125928.png]]
- an edge $(u,v)$ is added/deleted to/from the graph: here the message will have the same purpose as before but, 
	- here $h_v^{l+1}$ doesn't have any contribution from vertex $u$  when the edge is added, this mean that $h_u^{l-}=0$, it's a simplified case from the previous point
	- When and edge is deleted we have instead that $v$ shold not get any contribution from $u$ so $h_u^l=0$ this mean that $m_{ub}^{l+1}=0-h_u^{l-}$
In the message inbox a single vertex $v$ can be used to receive multiple messages from vertices at hop $l$, their are invariant to permutation  since all aggregations are commutative and can be aggregated in any order.

This was a case study done for the $\text{sum}$ operator but other linear operators follow aswell, for $\text{mean}$ and $\text{weighted sum}$ the update propagates a weight $\alpha$ to the neighbor, thje message to propagate $u$ to $v$ at hop $l$ is then $m_{uv}^{l+1}=\alpha h_u^l-\alpha h_u^{l-}$ with $\alpha$ being the in-degree of the sink vertex for $\text{mean}$ and edge weight for $\text{weighted sum}$.

## Complex aggregators

We discuss now the adding of complex aaggregators starting from attention bases architectures.
### Attention based aggregations
In a GAT  we have a learnable importance weights assigned to neighbors during aggregation, better than GCN, the hidden embedding is formulated at hop $l+1$ as
$$
h_v^{l+1}=\sum_{u \in N_{in}(v)}\alpha_{uv}^lh_u^l
$$
$$
\alpha_{uv}^l=\frac{\exp(z_{ub}^l)}{\sum_{u \in N_{in}(v)} \exp(z_{uv}^l)}, z_{uv}^l=a^l \cdot (W^lh_u^l||W^lh_v^l)
$$
where $a^l$ and $W^l$ , the coeffient $\alpha_{uv}^i$ depends jointly on embeddings of both source and sink vertices, changes to $h_v^l$ will trigger changes to downstream embeddings $h_v^{l+1}$ and reduces to recomputing thus losing some efficiency. However for all out-neighbors of $v$ at hop $(l+1)$ not yet been updated embeddings can be updated incrementally.
![[Pasted image 20260331151447.png]]
Here we see it better  whatr have said, the number of recomputations is higher for GAT w.r.t GCN-MAX, here we can analyze better how the incremental recalculation is done here $D$ changes thus we see how the change spreads across layers, if we adapt the updating rule here for $D$ we get
$h_D^{l+1}=(\alpha_{AD}^lh_A^l+\alpha_{CD}^lh_C^l)=\frac{\sum_u \exp(z_{uD}^l)h_u^l}{\sum_u \exp(z_{uD}^l)}$ . Now if you look at what happens when updating $h_C^{0-}$ to $h_C^{0}$  in $h_D^1$ only $\exp(z_{CD})h_C^0$ and $\exp(z_{CD}^0)$ are affected we just exclude them by sending two informations instead of one, for numerator and denominator
$$
m_{CD}^1=\langle (\exp(z_{CD}^{0-})h_C^{0-}-\exp(z_{CD}^{0})h_C^{0}),

(\exp(z_{CD}^{0-})-\exp(z_{CD}^{0}))\rangle
$$
This is an hybrid-incremental approach for GAT, any vertex $u$ that appears at hop $l$ of the propagation tree performs recomputation from hop $(l+1)$ onward, for vertices that are not yet updated we use incremental, like in the image above.

### Monotonic aggregations

Monotonic aggregations like $\max,\min$ cannot be updated with deltas.
![[Pasted image 20260331153433.png]]
Here we see an example, a change here is apparant only when the current hop is reached and recomputation is needed only if an update invalidates the current extrema's contribution. The hybrid incremental approach is extended here for this kind of function, a message sent from $u$ to $v$ , so $m_{uv}^{l+1}$ at hop $l+1$ contains both the new $l$-hop embedding and the old embedding of $u$ meaning $m_{uv}^{l+1}=\langle h_u^{l-},h_u^l \rangle$ is what $v$ receives.
A sink vertex at hop $l+1$ upon receiving such message will do one of three things:
- If the new embedding $h_u^l$ is identical to the current one, no update occurs. if $h_u^{l-}$ did not contribute to the current embedding of $v$ and the new embedding of $h_u^l$ doesn't either then the update from $u$ to $v$ is not relevant and thus not performed.
- An incremental update is computed at $v$ meaning theat $h_u^l$ is greater than the current embedding at $v$, so $v$ incrementally updates its embedding bu replacing the old contribution from $u$ wit the new one and aovid full recomputation.
- Recompute performed at $v$, the old embedding $h_u^{l-}$  contributed to the curernt $v$ embedding but $h_u^l$ no longer preserves  monotonic consistency required to perform an incremental update we might have that the new value is no longer the max across all neighbors to ensure correctness $v$ must recompute its embedding from all neighbors.
## analytical model

We compare RIPPLE++ over $\text{RC}$ , let these variable:
- $a_l$ the number of active vertices at hop $l$ in the propagation tree, i.e. vertices for which $h^l$ requires an update
- $\delta$ average degree of the graph 
- $d_l$ the dimensionality of the embedding at hop-$l$ 
For this assume a $\text{sum}$ aggregation function, upodating the embedding of $a_l$ active vertices at hop-$l$ involves, for $\text{RC}$:
1. Aggregating in-neighbors: the $h^{l-1}$ embeddings of in-neighbors of each active vertex are aggregated requiring $(a_l \cdot \delta \cdot d_{l-1})$ Flops
2. Compute: for embeddings of size $a_l \times d_{l-1}$ multiplyed by a weight matrix $d_{l-1} \times d_l$ will take a total amount of $(a_l \cdot d_{l-1} \cdot d_l)$ Flops
in total we get a cost at hop-$l$ of $(a_l \cdot \delta \cdot d_{l-1}+a_l \cdot d_{l-1} \cdot d_l)$ .

for RIPPLE++

1. Preparing and aggregating the message: active vertrices generated at hop $(l-1)$ are $a_{l-1}$ sending a message to its out neighbors sending a messag to its out-neighbors and aggregating it in their inbox. All of this require $(a_{l-1} \cdot d_{l-1} + a_{l-1}\cdot \delta \cdot d_{l-1})$ Flops.
2. Compute: Aggregate the messages to compute the mebeddings for hop-$l$ by multypling them by the weight matrix and adding the result to the current embedding which costs $a_l \cdot d_{l-1} \cdot d_l + a_l \cdot d_l$ Flops.
so we have the finaly cost of RIPPLE++ for $a_l$ vertices at hop-$l$ which is  $(a_{l-1} \cdot d_{l-1} + a_{l-1}\cdot \delta \cdot d_{l-1} + a_l \cdot d_{l-1} \cdot d_l + a_l \cdot d_l)$ that is approximated to $(a_{l-1} \cdot \delta \cdot d_{l-1}+a_l \cdot d_{l-1} \cdot d_l)$ .
Comparing the we get:
- RIPPLE++'s cost which is approximately $(a_{l-1} \cdot \delta \cdot d_{l-1}+a_l \cdot d_{l-1} \cdot d_l)$
- RC's cost which is approximately $(a_l \cdot \delta \cdot d_{l-1}+a_l \cdot d_{l-1} \cdot d_l)$ 
at each hp active vertices either expand or remain constant, $a_l \ge a_{l-1}$, the different in cost is simplified as $(a_{l-1}-a_l) \cdot \delta \cdot d_{l-1}$ . In case the frontier expands RIPPLE++ costs less otherwise the cost doesn't change so $\text{RC}$ cost is not exceeded.
![[Pasted image 20260331162137.png]]
Here we see that the number of operations done by RIPPLE++, we can see that the estimated operations and the actual execution time are heavily correlated and that at all hops number s RIPPLE++ takes considerably less time than $\text{RC}$ with a very good correlation of the model to the actual performances.
## Distributed GNN inference

The intial graph partioning and structure with features and embeddings are kept in  the worker machines memory, METIS is used to partition the intial graph so that the vertex count is balanced across workers and edge cuts across partitions are minimized with one partition per worker. Boundary halo vertices are duplicate and we use edgecuts to reduce communication during BFS propagation.

### The execution model
This is the full picture for the inference phase.
![[Pasted image 20260331163208.png]]
As we can see this is a worker-server with distributed execution, the server receives the incoming stream of events and router them to the relevant workers, it also maintains relevant metadata like 
- vertex to partition mapping $(V:P)$
- high-degree vertifes $(HE)$ 
for efficient routing.
Each worker stores the embedding for its local vertices and is reponsible for esecutring the lifecyle of its local vertices.
Using the inbox in a distributed setting might be memory-intensive so instead we use a managed pool inbox that is assigned to vertices on demand when a message is received, meaning that after processing a batch all inbbox are unasinged and reassigned to ther active vertices, this pool can be expanded dynamically on-demand and can reduce memory overhead from repeated allocation and right-sizes memory use. 
Workers don't hold vertices for halo embeddings but inboxes, these server as a proxy for sending message from local to halo vertices in a partitoon adn aggregate local partition using a MapReduce style $\text{Combiner}$ , after the local send operation in a hop we do a remote send of these aggregate messages to remote workers having their partition and on the receiver side an aggregation is done to stttart the next hop compute
### Request batching and routing

The servver receives a strem of both vertex-level and edge-level updates routed to appropriate worjkersm batches of updates are created and destinated for each worker if size $bs$ . The term $v_{ij}$ indicates and edge update from  $i$-th vertex to the $j$-th vertex. We always assume that at least one edge is added along with a new vertex., vertex deletions and feature updates apply only to nodes that are present.

A vertex update/delete is easy to route and its assigned to the wroker with the local vertex. The server maintinas a vertex partition map from each vertex to its partition, but edge additions or deletions may span workers.
- An edge request is completly local if both vertices are within the worker's partitions, the update is assigned to that worker
- An edge request spans workers if its source and sink vertices are local to different partitions, we assign the edge update to the worker and that has the source vertex of the edge update, and send a no-compute request to tthe sink vertex's worker toi update its partition with the new edge.
Vertex addition is troublesome and require more sophisticated handling as a new vertex to assign require considering placement quality  for load balancing an minimizing edge cuts similar to graph partitioning. RIPPLE++ offers to its users to configure custom routing algorithms:
1. a hash based routing, we has the vertex ID into a partition $(V \% P)$ 
2. locality aware routing which is discussed below as it is a novlety of this framework.
One the batches of size $bs$ the router routs them according to the selected policy.

For edge events we discuss a locality aware strategy, first we see the algorithm and discuss its implementation
![[Pasted image 20260331165535.png]]
- if both vertices are in the same partition this is and addition ora deletion $v_{12}$ is sent to that partition
- if the incident edges are on different partitions a cut edge is introduced between partitions $\pi(v_1)$ and $\pi(v_2)$ line $\text{CutEdge}$ routes the edge to the srouce vertex partition but send a copu to the other partition
- If only one vertex exists, $v_2$ m the other $v_1$ is new and is thus an addition. If the incident vertices of $v_{12}$ hav a high in-degree co-locating the edge with the vertex enhanced locality and reduces inter-worker communication for the propagation tree. We maintain a High in degree set, noted $HD$, at the server for this. iv $v_2 \in HD$ then we place $v_{12}$ in its partition otherwise we pick the partition with the least load
- if non of them exist we assign them to the least loaded partition to balance the vertex load

The HD set contains vertices with a degree greater than $2 \times (\textbf{average graph degree})$,.
### Update processing
upon receiving a batch of updates a worker applies the topology and featrues to its local partition and embeddings, depending on the update it prepares message of the hop-1 vertices,for halo vertices the mesasges ar combined in a local mailbox for the halo across all propagation trees at hop-0 once all vertices in this hop are processed by the worker the combined message are sent to matching vertices on remote workers before the next hop starts.

for all hops $l=1,\dots,L$ the workers aggregate messages in the vertex inboxes, compute the local embeddings,prepare the downstream messages and similar to asingle machine as before deliver and aggregate the local messages. The next hop starts when the prior hop are updated and incremental mesages are delivered to all workers, this is repeated L times till we get the final layer embeddings for the batch.

## Evaluation and baselines

In this setup they used numpy and MPI in python on both single machine and distributed setups.
Both CPU and GPU are supported.

Compared against InkStream.
reuslts are checked for ccuracy and determinism.

The used dataset with some statistics are the following

![[Pasted image 20260331172512.png]]

Used GNN models:
- GraphConv
- GraphSAGE
- GINConv
- GATConv
the aggragations used are sum,mean,weightesum,monotnoic with min and max and atention based
so we have
- GraphConv with sum $(GC-S)$ and mean $(GC-M)$, weighted sum $(GC-W)$, max $(GC-X)$
- GraphSAGE with sum $(GS-S)$
- GatConv with attention $(GA-A)$
- GinConv with sum $(GI-S)$


a random 20% of the vertices and edges are removed, the remaining 80%serves as the initial snapshot of the graph. on this 80% the models are trained and embeddings extracted that are the intial state of inference. The graph is partioned using METIS into the rquired number of parts and load the local subgraph their vertifes and halko vertices in memory.

In the paper they use DGL 
- vert-wise infernce on CPU called $\text{DNC}$ and CPU+GPU $\text{DNG}$ 
- its layer-wise recompute strategy $\text{DRC}$ and CPU+GPu $\text{DRG}$, we have that $\text{DRC}$ and $\text{RC}$running on CPU are the most competitive baselines.
### Single machine performance
![[Pasted image 20260331173936.png]]
![[Pasted image 20260331174809.png]]
RIPPLE++ consistently beats the baseline and DGL's version.
- increasing $bs$ gves minimal performance beenfits for $\text{DRC}$
- $(GC-W)$ has lower performance compared to pure incermenatal workloads
- The throughput speedup of RIPPLE over $\text{RC}$ decreases as $bs$ increases.
// todo

### Distributed performance
Here RIPPLE++ is compared against $\text{RC}$ , DistDGL does not allow online updates and Inkstream does not support distributed execution.
![[Pasted image 20260331175627.png]]
Scalability study w.r.t number oif partitions
![[Pasted image 20260331180009.png]]
