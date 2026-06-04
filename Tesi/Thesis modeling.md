
For the thesis we are about distributing a GNN to enhance performance, resilience to failures and scalability.

In our context we work on **streaming dynamic graphs**  meaning we have a graph that evolves over time we vertex and/or nodes being added/updated/removed over time.

In a formal setting at time $t$ we have $G_t=(V_t,E_t)$ starting from an initial graph $G_0$. We assume $t$ to be discrete and that the graph us updated in between two timeframes, for now we don't now anything about the distance between the timeframes.

The graph is updated through the following set of operations:
- $\text{AddNode}(u_i)$ :add node $i$ with its features.
- $\text{UpdateNode}(u_i,f_i)$: update node $i$ features with the new features $f_i$.
- $\text{RemoveNode}(u_i)$: remove node $i$, and its neighboring edges.
- $\text{AddEdge}(u_i,v_i)$ :add edge $i$ between two existing nodes $i$ and $j$.
- $\text{UpdateEdge}(u_i,v_j,f_{ij})$: update edge features for the edge between node $i$ and $j$ with the new features $f_{ij}$.
- $\text{RemoveEdge}(u_i,v_i)$: remove edge between between node $i$ and $j$.
Between two timestamps $t$ and $t+1$ updates are applied in batches from a batch of operations  $B_t=\{op_i,\dots,op_n\}$, so we get $G_{t+1}=UpdateGraph(G_t,B_t)$.

In our case the graph is used by a **Graph Neural Network** to predict informations about the graph itself using the capacity of the network to model complex relationships. In our case we care about the inference phase assuming we use a pretrained GNN, specically doing inference on a GNN can be done in various ways:
- predict information about the whole graph
- predict information about a part of the graph
- predict information about an edge or group of edges
- predict information about a node or group of nodes
For our case we focus on the latter a.k.a. **node level prediction** and try to use the GNN semantics to reach our performance objectives.
A GNN model, when doing node level inference, with $L$ layers is formulated as follows
$$
x_u^l=\text{Aggregate}(\{h_v^{l-1}|v \in N(u)\})
$$
$$
h_u^l=\sigma(\text{Update}^l(h_u^{l-1},x_u^l))
$$
with $l=1,\dots,L$ and the update function containing also the learned weights of the network.

For our purposes we assume the logic from RIPPLE++: we don't actually perform the inference of the final information but we care about having information of the dynamic graph being propagated as fast as possible in the network so that the embedding used for final prediction, the ones at layer $L$, change to incorporate that information. We monitor a set of nodes requested from clients that host a final model $f_{\theta}(h)$ that will perform the final inference of informations with a push based model to notify them of any changes. For our case we will then have at time $t$ set of requested nodes $R_t \subseteq V_t.\forall t$. 

Note: The set of request nodes might expand or shrink, for now we don't know or assume anything about the pattern of requests.

A GNN is very big and will not fit on a single computer, specifcally in inference we have that to perform inference on a single node all the various $h_u^l$ values have to be calculated up until the lat layer $L$ forming the **computational graph**. The computational graph is very big and won't fit in memory, in our case we will want all the graph always present in the main memory of our system as we don't know which nodes will be requested and we will need to distributed it across partitions. 

From RIPPLE++ we get that the layer-wise strategy seems to be the mostly controllable one: we only need to have the previous layer present in memory to computer the  layer of embeddings $h_u^l$ up until the last layer, and memory can be controlled better since we already know the size to be expected and works best when we batch requests.

Now we get to the last part: the partitioning strategy. In RIPPLE++ we see an approach that partitions the graph structuring around the vertexes:
We have $n$ partitions $P=(P^V,P^E)$ such that at each time $t$
- $P^V_t\subseteq V_t$ and $P_t^E \subseteq E_t$
- $E_t=\cup_i P_{t,i}^E$  and $V_t=\cup_i P_{t,i}^V$ 
The partitions might also not be mutually exclusive as for performance needs replicating nodes and edges is very useful and is also used by RIPPLE++ as a strategy: edges will be replicated across partitions to avoid unecessary communications.

We have a set of $m$ workers $W$, each worker will have its assigned partition and we have an emitter node $Em$  that distributes across partitions the nodes and also routes to the correct workers the updates on thegraph, for now we assume $m=n$ as we have a single partition of each worker.

Each worker for its assigned nodes will contain:
- the local worker edges
- the external workers edges
- the embeddings, at each level , for its assigned nodes

Depending on the partioning strategy over time partitions might become unbalanced leading to:
- a single worker having too work too many work to update all the nodes
- workers will have to communicate too much information between each other 
- Memory might also run out leading to OOM errors

Now this is very problematic since balancing for nodes will create problems as neighborhood degree has very skewed distributions meaning we have to somehow balance for node locality grouping by nodes communities to create inter-worker locality and avoid as much as possible interworker comunication that will slow down the system. This also has to happen on stringent latency SLOs which is the problematic issue as the optimal placement for nodes is a NP-hard problem.
The last problem is that even if we somehow manage get this to work out of course there is no perfect solution and nodes /edges might have to move between partitions to actually be able to rebalance the graph especially over a long time.

The open research question then becomes: how do we balance and/or rebalance the dynamic graph while keeping stringent latency SLOs?

For this RIPPLE++ offers a possiblity but achieving only a small number of edge cuts that give a very small gain of 3% at most over the hash based routing. The problem is then to model how the emitter node $Em$ behaves, we can see it as a simple function that depends on 
- a state being modified giving us a number corresponding to a worker/partition
- the current node 
- the current graph

$h(u,s_t,G_t)$ the internal state $s$ is also updated after doing doing the assignment $s_{t+1}=StateUpdate(s_t,u,G_t)$, note that this implies stateful stream computations, if we use a stateless approach we don't need $s$ at all.

The parallelization strategy adopted is BSP in our case.
- $S$ is the number of supersteps
- $h$ is the maximum number of messages being sent and received at each computation step
- $g$ is how many messages at a time the network can deliver s.t. $gh$ is what it takes to deliver $h$ messages of size 1 
$$
W+Hg+Sl=\sum_{s=1}^{S}w_s+g\sum_{s=1}^{S}h_s + Sl
$$
For our case a super computation step is an application of the $\text{Update}$ function and the communication step is the application of the $\text{Aggregate}$ function, and we have $L$ supersteps necessary for an update to "surface" on the last layer and influence all the embeddings it is related to in  a stream processing fashion.
