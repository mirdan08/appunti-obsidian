# Centrality
## Personalized PageRank
Notably Personalized PageRank is a modified version of PageRank which allows users to specify a starting vertex, $s$ and all random walks are restarted from the personalized vertex. 
The paper "Distributed Algorithms on Exact Personalized PageRank" proposes a theoretically perfect partitions using hierarchical methods. Since it's random we can employ the Alias an Alias tree variants to sample efficiently from large number of neighborhoods on paper "Distributed Algorithms for Fully Personalized PageRank on Large Graphs".



## Betweenness centrality

this measures the extent to which a specific vertex lies on the shortest paths between other vertices within a graph, the underlying premise is that important vertices are more likely to be on many off these paths, influencing communication fglow across the graph.
Given $C_b(u)=\text{normalized between centrality of vertex }u$ the most common approach is to use Brandes algorithm which calculartes $C_b(u)$ as follows:
$$
C_b(u)=\frac{1}{(n-1)(n-2)}\sum_{s \in V_ \subset \{u,s \}} \delta_{s^o}(u)
$$
the value of $\delta_{s^o}(u)$ is calculated recusrsively as follows:
$$
\delta_{s^o}(u)=\sum_{u \in P_s(w)}\frac{\sigma_{su}}{\sigma_{sw}}(1+\delta_{s^o}(w))
$$
with $P_s(w)=\{\text{predeccesors of w in a SSSP DAG \}}$, Brandes' algorithm is made of two steps:
1. SSSP involves creating a DAG from each vertex in the graph
2. apply recursion to compete dependcy score $\delta_{s^o}(u)$  and accumulate them to determine $u$.

For this algorithm the most improtant part is parallelizing across $n$ SSSP DAGs,. This means the brandes steps have to concurrently be executed on all vertices, and we can compute that in the BFS order when graphs are weighted instead Bellman-Ford and Lenzen-Peleg are used instead. This requires a lot of message passing to mitigate it we can use Crescenzi's algorithm which uses Bellman-Ford updates with differential values instead of full recomputation. And we can also employ an early termination strategy called "Finalizer" to stop the SSSP step as long as all vertices have received a  correct $\sigma_{su}$ which is effective for small gfraph diameteres. In this case messages regarding shrt distances are prioritized during communication as they are the most probable to change over a short period of time. 

## Closeness centrality

Quantigfies the shortes paath length from a given vertex to any other vertex in the network, this suggest that a vertex of higher iimprotance typically exhibits shorter average dfistances to other vertices the normalize closeness centrality is calculated as
$$
C_c(u)=\frac{n-1}{\sum_{v \in V}d(u,v)}
$$
With $d(u,v)=\text{shortest distance from u to v}$.

This has a very big overhead in communication, STREAMER addresses this using a level structure to filter out edges that do not impact shortest path distances DACCER then uses an approximation of the relative rank when vertices have hgiher closeness they also have larger neighborhoods with high-degree vertices using this each vertrtex can aggregate information from neighbors 

## Community detection

Louvain is the most effectgive but computationally intensive, while label propagation is the most effective one for real time purposes but won't always detect communities.

## Louvain
 This works on the concept of "modularity", which levareges the following equation:
 $$
 Q=\frac{1}{2m}\sum_{i,j}[A_{ij}-\frac{k_ik_j}{2m}]\delta(c_i,c_j)
 $$
 here we have various terms:
 - $A_{ij}$ represents the weight of the edge connecting vertices i and j
 - $k_i=\sum_j A_{ij}$ is the sum of the weights of all edges attached to vertex i
 - $c_i$ denotes the community to which vertex $i$ belongs
 - $\delta(u,v)$ which outputs 1 if $u=v$ else 0 
 - $m=\frac{1}{2}\sum_{ij}A_{ij}$ is the sum of weights of all edges
 Each vertex is a separate community in the beginning, the iteratively each vertex i is evaluated to gain modularity resulting from moving it to a neighboring community j. the vertex is placed in the community that yields the highest increase in modularity and such process is repeated until modularity doesn't increase anymore.

  Load balancing is cirtical for efficienncy of louvain algorithms in distributed enirnoments, the graph partioning methods deal with high-degree vertices that heavily skew the workload. This canb be extended to in various ways one proposal is to replicate high degree nodes like in 
  "# A Scalable Distributed Louvain Algorithm for Large-Scale Graph Community Detection", the methods replicates across all machines  ub tge systesm and the edges connect to these verticews are reassinged to ensure a balanced distribution of edges.
  
Reducing communication overhead is done in a few ways, some studies propose ghost vertices which are a replicas of of the vertices connected to local vertice but processed by other machines.

## Label propagation
Is an effective approach to detecting communities by propagating labels throught the network, the algorithm starts by assigning a unique labels to each vertex representing the community to which the vertex currently belongs, in each iteration every vertex consdiera the label of its immediate neighbors and adopts the label that is most frequent among them the algorhtm contiues to iterate thorugh the vertices and update their label until convergence. In this case misordred computation result in large impractical communities the EILPA algorithm identifies inluential vertices using a personalized PageRank model, label propagation starts from these leader vertices enusring structured ingluence based community detection.The PSPLAP algorithm consdiers vcertex influenec by assigning weights via a k-shell decomposition and calculating propagation probabilities and vertex similarities based on these weights. Vertices update their labels during each iteration  using these probabilities and simialrities and aiming for more coherent community dtetection by reducing label updating randomness typical in LPAs. Communication overhead is another prevalent challenge when impementing the LPA in distributed enrivonemnt a dynamic mehcanims alternating between push and pull models is used to reduce communication costs, initially push is used for rapid community formation and then the algorithm sdwitches to a pull in later itreations to considera neighbors label quality.

## Connect components

this compuytation is usually done with a BFS or DFS traversal, with linear time complexity but for distributed environments  the irregularity of the graph leads to centralized workload and communication in hgh degree vertices, using a max consensus like protocol  where each vertex considers the maximum value among its neighborts to update its own values the algorithm can termine in $D+2$ iterations with $D$  being the diameter of the graph,  benefitting fromn this protocl each vertex only need to rteceive information from its in neighbors which also helps improve computational and communication efficeincy.

Balancing the workload is impotant for improving the fificiency of connected component computation, high-degree vertices are replicate to different machines and we dynamically redistribute the vertices 

- "Communication-efficient Massively Distributed Connected Components". replicate high-degree vertices
## Similarity

This approach is highly parallelizable and check whether two nodes are highly similar or not. 
The first known approach is Jaccard similarity for two sets
$$
Jaccard(A,B)=\frac{|A \cap B|}{|A \cup B|}
$$
we also have a variant of this  called cosine simialrity
$$
Cosine(A,B)=\frac{|A \cap B|}{\sqrt{|A|\times |B|}}
$$
In our case we use the sets contining all the nieghbors of a given vertex. The communication overhead poses a significant challenge  especially when computing similarity values between all vertex pairs. In this case WHIMP calculate the incidence vector of vertices in the ghraph to identify all pairs wgiose cosine similarity exceeds a certain value.
SimilarityAtScale  adopts the method fo compressing vertex to reduce the volume of communication between mahcines during the computation of jaccard similarities.

Another approach is SimRank, the simialrity of two objects is determined by its neighbors similarities formally defined as 

$$
s(a,b)=\frac{C}{|I(a)||I(b)|}\sum_{i=1}^{|I(a)|}\sum_{j=1}^{|I(a)|}s(I_i(a),I_j(b))
$$
$C$ is a constant between 0 and 1 and set of in neighobrs of $a$ is denoted by $I(a)$ with $I_i(a)$ being the ith in neighbors  usually ywo algorithms are used
1. iterative algorithm: the algorithm iteratively computes the SimRank scores accordiong to its definition until convergence or a fixed number of iterations
2. Monte Carlo it uses a random walk based algorithm where SimRank score between two vertceis is based on the expected first meeting time of random walkers starting from each vertex and traversing the graph in reverse
In this case otpmizaing parallelism is important as it involves random wlaks, each wlak simulation is independet and suitable for parallel processing. 
DISK constructs forests instead of random walks thus each vertex can build its layer of child vertices, CloudWalker the recursive computation of SimRank is transformed into sovling a linear system by estimatiing a diagonal matrix which represent attributes of each vertex.

to balance the workload DISK declares two laod thresholds in machines with high load above the limit will be moved to low-load machines. Many papers adopt data sharing to reduce communication costs overhead.

## Cohesive subrgraph

In this part they focus onn three popuelar cohesive subgraphs: k-core,k-truss and maximal clique by definition the level of density we have that k-core < k-truss < maximal clique, but their computational efficincy is in the reverse order.

The k-core ofa graph is the maximal subgraph in whic each vertex has a t least a degree of k. the core value of a vertex is the highest k for which it is part of k-core and the decomposition algorithm is usually used to compute k-core uses the iterative approach to compute these values for all vertices starting with a specified k and decrementing in subsequent iterations, the algorithm functions by deleting vertices with degrees less tthen the current k while updating the degrees of the remaning vertices , this continues until vertices are assigned to a core value marking the completion of the computation. Parallelism poses a challenge in k-core decomposition of algorithms especially for the vertex delition approach: each removed vertex causes sequential update to its neighbors. the deleting and updating phases are then decoupled for parallelization which is faster but the ddeltion phasdes might ignore conflicts and and updating adjust the degrees this might require more iterations. 
To minimize communication overhead the subgraph centric approach is widely adopted each vertex doesn't boradcast message to all neighbors but selectvely only to those whose coreness value are anticipated to change.

(might be useful in RIPPLE++?)

using an approximate k-core decomposition algorithm impsing an iteration limit it is demonstrated that for $\gamma >2$ we only need $\lceil \log{n}/{\log(\gamma / 2)} \rceil$   iterations to reach an appeoximate core value for all vertices at most $\gamma \times \text{(actual value)}$ , in each iterations it uses multiuple thresholds to expedite the process and removes verttices whose deggree falls below the highest thresholds morevoer each vertex falls below the highest threshold. each vertex communicates only weith a specific subset $\Lambda$ which reduces the communication volume to $\log_2 |\Lambda|$ .

The k-truss is a subgraph where each edge is part of at least $(k-2)$ triangles , as fore k-core iteratively we delete edges that are part of fewer than $(k-1)$ triangles