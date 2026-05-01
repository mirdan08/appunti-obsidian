+k-way based partioning algorithms for processing large streaming graphs usually ignore the neighborhood when splitting thegraphs to focus insted on avoiding vertex replication. The paperproposes a novel optimzation goal that aims at maximizing the number of local triangles in the paritions as and additional objectives. 
Triangle count is an effective metric to measure conservation of community structure, here we see a prposal for a family of heuristics ofr online partioning over and edge stream using three DSAs: Bloom filters, Triangle Maps and High Degree Map.

# Introduction
Graph partioning can be a vertex-based k-way partioning  placing each vertex on one of k partitions s.t. each has a similar number of ertices to achieve load balancing and edge-cuts between vertices in different partitions are minimized. On the other hand edge-based partioning places each edge on one of the partitions to balanche the edges per partition and minimize the vertex replicas across partitions. For power-law graphs edge-based is more effective as it is seen in rel-world graphs and is the focus of this article.

Optimal graph partioning is NP-complete and many heuristics hae ben proposed for static graps, but modern day graphs are streamed and dynamical. Streaming partioners always assume a central machine  which decides the partition on which to place the graph element and worker machine receive updates from the leader and add to their local respective partition.

Each arriving edge is represented as vertex-pair from the update stream and is placed in one partition and a vertex is replicated when its incident edges go to different partitions to those partitions. Below we have a figure repesenting the situatuion better:
![[Pasted image 20260429113359.png]]



Formully our objetive is to achieve edge-balancing by getting to more or less $\frac{|E|}{k}$ edges per partitions and mnimize replicated vertices with a **vertex replication factor** stated as 
$$
\rho=\frac{(\sum_1^Vr(v))}{|V|}
$$
with  $1 \leq r(v) \leq k$ is the number of replicas per vertex (up to k).

Blancing edges per partitons helps load balance the compute and memory load on workers when executing distributed graph analysis ove the partione fraph and reducing vertex replication mitigates duplicate elements across partitions and reuces messaging between them during distributed execution .

The leader might use local in-memory state to make the decision of edge placement but given the large graph iszes it is important to have a state s.t. it is bounded to $O(|V|)$ and also $<< O(|E|)$  this is the formalization of a good and balanced partition graph. We should preserve the local community structure too within each partition as this is very good for GNN computation graphs.

The most common case is real-world graphs typically exhibit a power-law degree distribution and small-world properties with few highly connect hub vertices forming dense communities and sparse inter-community links. Community quality is measure with Local Clustering Coefficient (LCC) and measures how close a vertex's neighbors are to forming a clique helping identifying small-world networks.

Several edge-based streaming partitions with edge load balancing and min vertx replica objectives exist but do not care for the community structure being preserved.

In the paper the proposal is to:
1. Formalize the problem ofedge-based streaming partioning to enhance the local community struture and discuss the benefit of using triangle preservation to retain community structure and define a novel objective functon forthe same 
2. A multithreaded partioning fraework *TriParts* is developed to use compact data structures matinated at the leader to make partioning decisions
3. Four heuristics that incremntally leverage these states topartiton the incoming edge stream  tomeet our qualitative objectives
4. Copare Triparts to othe SOTA baseline for large real-world graphs  and five synthetic graphs and five syntethic graphs 
Static graph partioning is not interesting if not as a starting scenario lets swtich to streaming graph partioning, first we  distinguish into stateless partion algorithms,using hash functions, and stateful ones that have an internal incremental state that is updated each time.
The steram can be random or according to a BFS/DFS.

In **Vertex based partioning** these systems process a stream of vertices with their adjcency list.
- LDG penalizes large parttions by placing vertex where partitons have the most neighbors
- FENNEL balances neighbor co-location with non-neighbor separation 
- CUTTANA reduces replcation by buffering the graph to retain global context before partioning
In **Edge based partioning** :
- PowerGraph proposesa  greedy algorithm to place an incoming edge onthe partitons where its incidnet vertces a been placed earlier it ofers a distributed implementation and maintains a $O(|V|)$  size for the state for decsion making at the leader, the most simple heuristic is using bloom filters to approimate this taking $O(c \times k)$ space complexity for a k-way partioning where $c$ is large constant representing bloom filter bits and $c < |V|$
- Grid and PDS are stateless hash-based heuristics with an upperbound on vertex replices but do not care for graph topology
- ADWISe proposes a window based algorithm that trades-off quality and latency
- CuSP partitions graphs maintained in distributed memory of large HPC systems.
- Degree Based Hashing and High Degree Replicated First are SOTA edge-based streaming partitioners are SOTA edge-based streaming partioners
	- DBH maintains the degree of both vertices for each edge using a hash table of size $O(|V|)$ 
	- HDRF replicates high-degree vertices to reduce replication and balances load using a scoring function but communication all partitions pe edge leading to a high overhead
The proposals use a larger state at the leader than HDF but smaller then DBH, the leader comunicates wit hthe partitions periodically toupdate its state,time and space complexities fall in betwen the twotechqnieus , DBH doesn't consider graph structures at all while HDRF is optmized only to reduce the replication factor ignorin graph topology.

Hybrid tehcniques exist in the literature:
- PowerLyra combines both edges and vertices to et hybrid cuts for partioning based on the vertex degree
- Leopard performs vertex duplication with edge cuts and dynamic rebalcning of the partitions when elements are deleted
Note: here they consider an append only graph


Suppose the classical graph fomrulation $G= \langle V ,E \rangle$ , $|V|=n$ and $|E|=m$ , the stream is a series of undirected edges arriving at the leader machine $S=[e_1,\dots,e_m]$ each edge appears exactly once with an arbitrary arival order. 

## Problem formulation

the formal goal is todivide the edge stream $S$ into k subgraphs $\mathbb{S}=\{S_1,\dots,S_k\}$ each formed from a subset of the streams as the edges arrive, an arriving edge is placed on exactly one of the worker machine $W_i$ that holds the partition $S_i$ i.e. $\forall i,S_i \subset E, \cup S_i = S$ and $\forall i,j. S_i \cap S_j = \emptyset$   so partitions never overlap with a given a priori static $k$.

The first goal we have to achieve high quality streaming is is *edge balancing* meaning that $\forall i.|S_i|\approx \frac{m}{k}$ to allow balanced loading on the partitions each worker handle an evenly split load as the partition size grow built by scaling up CMs, when dierent edges incident on the same vertex $v$ are sent to different partitions if the number of replicas ofr a vertex $v$ is $r(v)$ and its consequent replication factor $1 \leq \rho = \frac{\sum_{v \in V}r(v)}{|V|} \leq k$   , $\rho$ is to be minimized as a secondary goal.

## preserving communities

With *Local triangles* we mean 3-cycles of edges that are fully present within a partition, for the *i*-th partition $\tau_i$ is the number of such structures. The *global triangle count* $\bar{\tau}$ is the set containing all the triangles in the unpartitioned graphi.e. $\tau \leq \bar{\tau}$ , we understand that partitioning eliminates triangles the toal umber of triangles then is $\tau = \sum_{i \in k}\tau_i$ and the ratio of triangles is also preserved $\hat{\tau}=\tau / \bar{\tau}$  .
We can present the hypotesis of traingles preserving the communities
### The community preserving hypotesis for triangles

Vertices within a comminity are more connected with vertices inside and less with nodes outside, an edge is between two vertces is in the same community with probability $p$ and proability $q$ of an edge to connect different communities. By definition $p >> q$ the expected number for a comunity $C$ with $n$ vertices is $(T_{in}^C)=(^nC_3)\times p^3$ .
the probability of a triangle existin across commuinties is either:
- $(T_{out})=(^nC_3)\times pq^2$  if two edges are in one community and the third in a different one  
- $(T_{out})=(^nC_3)\times q^3$ if three edges are spread across three communities.
<<<<<<< Updated upstream
Since $q << p$ and $P(T_{in})>> P(T_{out})$ , i.e. the chance of triangles forming a community is pretty high thus preserving suc ch structure helps creating communities.

Triangle counting is the basic building block of many community detection algorithms and is well studied and is also improtant to form complex networks with an underlying commuinity structure,
### Vertex replication hypotesis
If two partioning methods have similar vertex replication $\rho$ and edge balancing the one preserving more triangles $\tau$ and will have a high Local Clustering Coeffiicient(LCC).

For $\tau(u)=\text{\# triangles for which }u \text{ is part of in partition } i \leq k$ and $d(u)$ its degree, The LCC of vertex $u$ in partition $i$ is $lcc_i(u)=\frac{2.\tau_i(u)}{d(u)(d(u)-1)}$ and the average clustering coefficient is then $\bar{lcc}_i=\frac{\sum_{u \in V_i} lcc_i(u)}{|V_i|}$ .

Both partioning methods are edge-balanced each partiton will have then $\approx \frac{|E|}{k}$ edges and since both have the same vertex replication factor, the number of vertices for each partition will be more or less $\approx \frac{\rho|V|}{k}$, amd tje average degree of each partition wil also be the same hence if the tirangles preerved  in one partition $\tau_i$ increase also $lcc_i$ increases assuming a similar degree distribution $d(\dots)$ for vertices in the partioned graph under both methods.

If $\rho$ decreases this will cause LCC to decrease since trhe average degree in a partition will increase.
### Tirangle preservation hypotesis

We define the transitivity of a graph as $$Trans=\frac{3\tau}{|triplets|}$$ where $|triplets|$ is a set of three vertices connect by two or more edges.

Triangles contribute 3 times to the size we multiply by three the actual number.

A secondary definition is 
$$
Trans_=\frac{3\tau}{3\tau+|o\_triplets|}
$$
where $|o\_triplets|$ is the set of open triplets having a path but not ofrmin a triangle.

Suppose to have a partionare $M$ preserving hte number of triangles with sensitivity $$
Trans_i=\frac{3 \tau_i}{3 \tau_i + |o\_triplets_i|}
$$
For a single partitition $i$.

Suppose to have an alternative partioner $M'$ for that same graph giving the same number of ertices and edges in a partition but not preserve as many triangles, its transitivity is then defined as 

$$
Trans'_i=\frac{3\tau'_i}{3\tau'_i+|o\_triplets'_i|}
$$
for a partition i, with $\tau \ge \tau'_i$. Now we get also that $Trans_i \ge Trans'_i$ , suppose $T(u,v,w)$ be a triangle preserved in partition $S_i$ of $M$ but not in partition $S'_i$  of $M'$  then for $T$ we can distinguish two cases:
1. two edges of $T(u,v,w)$ are present in partition $S'_i$  so the denominator of $Trans'_i$ increases by 1 due to T and the numerator remains the same so $Trans'_i$ decreases 
2. only one edge is present in $S'_i$ , the transitibity of the partition is not affected by $T$.
Every missing triangle might decrease or mae the transitivity of $S'_i$ remain the same , in a fraction if the numerator and denominator are increased by the same quantity the fraction increases. Since $\tau > \tau'$ $S_i$ and $S'_i$ have the same number of edges and ertices we have $Trans_i \ge Trans'_i$ .

## Optimization problem

Preserving these structures is essential,we define this as an optimization problem with also our objective.

We want to partion the community coming from a stream $S$ of edgs into $k$ partitions s.t.:
1. Balance across edges is within a load-balancing factor $\epsilon$ $$\forall i. |S_i|\in[(1-\epsilon)\frac{m}{k},(1+\epsilon)\frac{m}{k}]$$
2. Minimize the vertex replication factor $\rho$
3. maximize the tirangle count ratio $\hat{\tau}$ 
For an input graph $G=\langle G,E \rangle$ generated "uniformly randomly" where we have probability $q$ of generating an ege we assume the largest clique in the graph to be of size $\tilde{n}$ .

When using an edge-based partioner for $\rho > 1$ some triangles will be missed $\hat{\tau} < 1$ an edge that is part of a clique and is replicated can break one or more triangles in the clique. 
For $K \in G=\langle V,E \rangle$ being a clique of size $\tilde{n}$ and $\tilde{V}$ its set of vertices, $K$ has ${\tilde{n}(\tilde{n}-1)}/{2}$  edges, the number of unique traingle is $^{\tilde n}C_3$ and each edge is part of $(\tilde{n}-2)$ triangles per clique.
When a edge misses we might have up to $(\tilde{n}-2)$ triangles, replicate a vertex within $K$ results in a missing triangle in a missing triangle with probability $\sigma$ over the entire stream, for a replication factor fo $\rho$ we get $\rho -1$ extra copies of a vertex and we get up to $\sigma (\rho -1)(\tilde{n}-2)$ triangles from the clique, if an edge $e(v_i,v_j) \not \in K$ s.t. $v_i \in K$ and $v_j \not \in K$ is assigned to partition $S_{k \not = l}$ it will not result in a missig triangle.

For a replication factor $\rho > 1$ a fractionfo triangles are boun to be missed in k-way partitioning of the graph irrespective of the partioning strategy. The factor $\sigma$ is not central to the argument and gets ignored.

## vertex replication factor

A high vertex replication factor will cause higher number of triangle to be missed, since the clique has $\tilde n$ nodes and edge is adjacent to 2 vertices , the loss in total triangles due tro vertex replication is bouded by $((\rho -1)(\tilde{n}-2)(n/2))$ triangles at most i.e. $O((\rho -1)\tilde{n}^2)$ triangles are lost.
Suppose $N(\tilde n)$ is the number of cliqus of size $\tilde n$ in thegraph, since the probability of anedge being present in the graph is $q$ and the cliques must contain all the edges all $O(\tilde{n}^2)$ must be presnt, the probability ofa clique of size $\tilde n$ is $q^{\tilde n^2}$  so the number ofcliques of size $\tilde n$ is $N(\tilde n) \approx O(|V|^{\tilde n} q^{\tilde n^2})$  and $O((\rho - 1)\tilde{n}^2N(\tilde n))$ triangles will be missied for al cliques of size $\tilde{n}$ due to k-way graph partioning with a replication factor $\rho$, the expected umber of total triangles missing in the graph $T_{miss}$ is then
$$
T_{miss}=\begin{equation}
O(\sum_{s=3}^{\tilde n}((\rho -1)S^2N(s)))\approx O(\sum_{s=3}^{\tilde n}((\rho -1)s^2|V|^s{q^s}^2))\approx\frac{(\rho -1)|V|q}{q^4(1-q^2)^3}
\end{equation}
$$
The total number of expected triangles is then 
$$
T_{tot}=\sum_{s=3}^{\tilde n}(^sC_3N(s))\approx O(\sum_{s=3}^{\tilde n}s^3N(s)) \approx \sum_{s=3}^{\tilde n}(^sC_3|V|^s {q^s}^2) \approx O(\frac{|V|q}{q^6(1-q^2)^4})
$$
Finnaly we can derive the fraction of missing triangles for $q << 1$
$$
\frac{T_{miss}}{T_{tot}}=O((\rho -1)q^2(1-q^2)) \approx O((\rho -1)q^2)
$$
The fraction of missing triangles is higher of a high replication factor in edge-balanced partioners, further a given replicationfactor thefraction of missing trinagles is more the graph is dense. The theory of grah partioning i.e. the retained comunities, can be improved due to the number of ranles preserved by improvinghte replication factor, so miniimzing replication factor and increasing triangle counts are not contradictory. but optimzing a vertex replication factor does not yield automatically the increased triangle count.

# System architecture

We use a leader machine and k-workers for a k-way partioning  wit the architecture below
![[Pasted image 20260430154726.png]]

While by default each worker is on a separate machine we can confiure multiple workers per machine. The lader receives an input stream of edges from an external source which is accessed thorugh a FIFO `ReaderQueue` from an external source which is accessed thorugh a FIFO reader queue by the leader. The leader partioning logic decides the partitionto which each edge should be assgine to and then sends that ege to the worker hosting such partition, the leader exploits concurrency usinga compute thread pool whose threads can access the read queue and in parallel decide on the placement of edges with high throughput.

This can have some impact on the quality of partioning but this amortized across thelong stream of edges except adversarial scenarios, it also has k transport queues on per worker to send edges assigned to them asynchronously  the leaer than uses three datstructures a Bloom filter per partition, a Triangle map and a high degree map.

Each worker will have a thread pool to handle the stream of edges asigned to it by the ader it addste received edges assigned to it by the leader it adds the recived edge ot the local partition maintained in-memory as an adjcency list, it also maintains a local triangle map data strucutre the number of threads in the leader computer and the owrker thread pools are configurable.

## Processing workflow

The reader queue is unbounded populated from the incoming edge stream, it is a thread safe java `ConcurrentQueue` and uses atomic operations allowing all the computer threads to concurrently remove the eges from the queue , eah thread indepednely esecutes the partioning heurstic using the local state information to assign the edge ot one of k partitions. 