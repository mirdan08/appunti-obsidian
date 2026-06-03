The paper focuses on the storage techniques and methodologies for graph processing, analytics and pattern matching

![[Pasted image 20260516110415.png]]

Multiple papers are cited, the most promising ones are :
- LSGraph

Parallel computing and scheduling are essential for efficiently processing large-scale dynamic graphs, they have high variability in the spatio-temporal sense thys static strategies are not suitable, we can apply a static graph processing or adopt incremental computation models.

*Evolving* graphs are not the same as *streaming* graphs,  evolving graphs are analyzed with perodic snapshots other recent works try to tackle this, among them one seems useful.
- IncBoost 
Current dynamic graph processing suffers in efficiency and scalability. The proposals of the paper are:
1. A hierarchical taxonomy:  a broad review of dynamic graph processing with a systematic taxonbomy categorizing existing approaches intop multiple distinct categories
2. A comprehensive review based on the proposed taxonomy
3. A review of the works with key technologies for dynamical graph processing
4. identify key limitations and potential directionbs
this is the organizatrion of the paper chapters:
![[Pasted image 20260516120154.png]]

## Modeling the graphs

First we define their notations and concepts $G(V,E,W)$ where $V=\{v_1,\dots,v_n\}$, $E=\{e_1,\dots,e_m\} \subseteq \{V \times V\}$ and $W=\{w_{e_1},\dots,w_{e_m}\}$ edges change via insertion delition or modification. An evolving dynaimc graph consistits of multiple snapshots  sampled ata ragularly spaced intervals $EG=\{G^{t_1},G^{t_2},\dots\}$ with $G^{t_1}=\{V^{t_1},E^{t_i},W^{t_i}\}$ being the graph at snapshot $i$ .
A streaming graph is represented as $SG=(G,O)$ $G$ is an initially static graph and $O=\{\sigma^1,\sigma^2,\dots\}$ is the set of operations changing theedge and vertices for which $\sigma^i$ is a tuple of kind $(op\_type,element,timestamp)$ with $op\_type \in \{+(\text{addition}),-(\text{removal}),*(\text{update})\}$ , $element$ specifies the corresponding edge or vbertex and timestamp indicates the time of the operation. Vertex delition is modeled by removing all edges incident to $v$ .

## Representing the graphs

The Adjacency matrix(AM), is the simples graph representation,suppose $|V|=n$ and $|E|=m$ then for this representation we take $O(1)$ to find an edge but has a cost of $O(n^2)$ in space overhead, it is memory ineficient for sparse graph but with GNNs and sparse matrices/vectors being used it is being adopted usin a Compressed Row Spared (CSR) is an optmized AM the matrix becomes three arrays: 
1. row offset: the starting indexx of each vertex neighbors in the adjacnecy list
2. adjancey list: neighboring vertices  for all vertices
3. value list holds edge atribute information such as weights 
The space overhead of CSR is $O(m+n)$ and given the edge degree $d$ the time overhead is $O(d)$  it is an afficient and cache firendly layout for graph analytics but poorly suited for graph update as they require quite a lot of data movement.

The classical representaion si Adjacency list (AL) combining vertex array with linked lists to store sparse graphs. All vertices are within a one dimensional array with each vertex's neighbors maintained in a linked list with flexiblity possile for varying vertex degrees. For AL the space overhead is $O(n+m)$ insertions take $O(1)$ and supports edge deletions with a complexity of $O(d)$ here $d$ is the average degree of the target vertex. For dynamical graphs this is pporly suited since it realies on pointers with addional memory and fragmented memory layouts.

Edge List (EL) is a widely adopted foramt for representing original graph data which stores the graph as a collect of edges $e(u,v,w)$ with $u,v$ and $w$ denoting the source  vertex,destination vertex and weight. The space complexity is $O(m)$ and locating a specific edge takes $O(m)$ it is suboptimal for storage efficiency and search but it uses a contiguous memory layout with efficient block loading and reduces random I/O making it well suited for out-o-core graph processing systems. 

## Processing the graphs

dynamic graph processing typically consists of two stages: graph updates and graph computations.
1. the update stage means the system integrates new edges from incoming batches into the graph structure 
2. the computation stage where analytics are performed on the update graph
These stages alternate iteratively to support continuous updates an analysis, dynamic graph processing generally falls in snapshot and streaming approaches. Evolving graphs use snapshots analyzing teh whole graph at discrete time interavls, this methods simplifies implemntation an suits offline historical analysis but introduces higher latency. THe streaming apporahc incrementally processes continuous updates such as edge and vertex insertions or deletions while preserving real time computation.  The latter one is more complex and better suited for scenarios demanding real time monitoring.

## Algorithms for graphs
the algorithms for graph fall into 6 categories:
1. path analysis
2. centrality measurement
3. community detection
4. subgraph mining
5. graph representation learning
6. graph neural networks

The paper focuses on categories 1 to 4. 
1. for path analysis applying a BFS and Single-source shortest path is essential
2. centrality measurement for measures like Degree centrality, closness centrality, betweenness centrality and PageRank
3. community detection algorithms mean finding densely connect nodes like Louvain method or Label Propagation 
4. Subgraph minig aims to identify structurally occuring or significant local patterns within the graph like motif statistics, triangle counting and k-truss decomposition
## Graph processing challegens
There are various challenges to be discussed for this kind of graphs.
1. Store challenges: dynamic graph storage faces several challenges, demand increases rapidly over time
2. Computation challenges: dynamic graph analytics present several key challenges that can be resumes as many interdpedncies between nodes andd cahnges might break or add them forcing continous recomputations that require agSgregating irregularly distributed data.
3. Scaling is also a big problem as largevelumes and heavilty skewed distributions  of dynamic graphs.
## Design principles for dynamic graph processing

![[Pasted image 20260516144932.png]]
these are the core pinciples related to graph processing
1. Space effiency: compressing and string vertex and edge attributes in a graph most of the structure remains unchanged  across adjacent time points. To efficiently store multiple data versopms redundancy is minimized by identifying  and sharing duplicated data across time thereby reducing storage overhead and enhancing the space efficiency of dynamic graph storage
2. Principle of good locality given a power-law distribution, dynamic evolution and irregular strucutre of dynamic graphs along with varying application requirements efficient data layout is essential for improving data locality an enhancing overall processing performance the design storage layout should align with application workloads  update and computation patterns and hte memory and cache hierarchies of the target hardware. Multiple storage foramts are used in this case to optimize organization of vertex and edge data. A well designed layout can significantly enhance both update efficiency and analytical performance enabling scale high perofrmance dynamic graph processing.
3. Principle of efficient indexing : graph data in real-world applications is characterized by highly frquent insertions,deletions  and queries of vetices and edges without an efficeint index mechanism these operations can incur in substantial overhead during updates significantly degrading perforamcne of dynamic graphg update queries and analytics. To address this we should use fast multi-dimensional indexing spanning vetices edges and tempora ldmensions emphasizing incremental update friendliness and strong data locality to ensure efficient access and minimal data movement. indexing mechanism should be adaptive to vetcies of varying degrees in addition to meet the demands of high concurrency and system scalability it shold incorporate topology and concurrency strategies to ensure efficient and consistent access in paralle and distributed environments.
4. Data consistency: maintaining consistency ina dynamic graoph requires a transacton mechanism within the storage system to uphold the ACID properties of operations. Preventing conflicts and cinossitencies from the conccurent access is critical for the reliablity of the system. also a version control mechanism should be employed to comrpehensively recorx the historical evolution of graph data and supporting data traceability and recovery
## Optimizing dynamic graph processing
We go into the desirable objectives to aim for
1. Low latency: use incremental computation, caching , precomputation and caching reduce end to end latency
2. High parallelism task parallelization and load balancing are crucial.
3. Good scalability is fundamental criterion in dynamic graph processing 
### Optimizing dynamical graph storage
the storage methods do not work well and are often inadequate.
![[Pasted image 20260516155003.png]]
This represents for each method the evolution over time, we distringuish CPU and GPU platforms
1. For CPU have more optimization we dwelve into different formats
	1. CSR-like structures mitigate high reconstruction overhead associated with tradiational CSR or array representations during graph updates while preerving their high perfomance graph analytics.They support efficiently dynamic updates while maintaining compact storage and high access locality.