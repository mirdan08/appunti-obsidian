Usually when we use GNNs we just care about finding an input-output mapping for the values, here instead we rain state-of-the art-GNNs to imitate individual steps of classical grapha algorithm such as BFS,bellman-ford and sequential algorithms like Prims's algorithms.
Hypotetically maximisation-based MPNNs are the best since graph algorithms rely on discrete decisions within neighborhoods and this claim is validated empirically.
A lot of algorithms share subroutines and we can demonstrate that we can have a positive transfer between tasks showing how learning  a shortest path alogrithm can be improved when learning simultaneously while providing a **supervision signal**.
A supervision signal is driven by how the algorithm would process such inputs and its relevant intermediate outputs providing explicit guidance on how to tackle graph structured problems, this learning approach is called **neural graph algorithm execution**.

## graph algorithms
we consider graphs $G=(V,E)$ with $V$ being set of nodes and $E$ being the set of edges, a GNN receives a sequence of $T \in \mathbb{N}$ , each element in the sequence is a graph with the same structure but varying metadata vor nodes and eedges.
Each node $i \in V$ has $\vec{x}_i^{(t)} \in \mathbb{R^{N_x}}$, each edge $(i,j)\in E$ has associated edge features $\vec{e}_{ij}^{(t)} \in \mathbb{R}^{N_e}$ with $t \in \{1,\dots,T \}$ , $N_e$ and $N_x$ being the respective dimensionalities for the edge and node features.
At each step the algorithm produces node-level outputs $\vec{y}_i^{(t)} \in \mathbb{N_y}$ , some parts of the output might also be reused for the next input i.e. $\vec{x}_i^{(t+1)}$ 
## learning to execute grap algorithms
For specific algorithm $A$ an input is provided to the network, the network follows the encode-process-decode paradigm.
1. encoder
For each algorithm $A$ we define an encoder network
$$
z_i^{(t)} = f_A(\vec{x}_i^{(t)},\vec{h}_i^{(t-1)})$$
$z_i$ are the encoded inputs produced using the previous latent features and and the current node input with $\vec{h}_i^{(0)}=\vec{0}$.
2. process
The inputs are then processed to a processor network $P$ , this network is not algorithm specific and shares its parameters among all algorithms being learnt. The network uses current the encoded inputs $\bf{Z}^{(t)}=\{\vec{z}_i^{(t)}\}_{i\in V}$ , the edge features $\bf{E}^{(t)}=\{\vec{e}_{ij}^{(t)}\}_{e\in E}$   and produces latent node features as output $\bf{H}^{(t)}=\{\vec{h}_i^{(t)}\in \mathbb{R}^K\}_{i\in V}$ .   
$$\bf{H}^{(t)}=P(\bf{Z}^{(t)},\bf{E}^{(t)})$$
3. decode
finally we can calculate the output $\vec{y}_i^{(t)}$ 
$$\vec{y}_i^{(t)}=g_A(\vec{z}_i^{(t)},\vec{h}_i^{(t)})$$
4. terminate
In order to terminate the execution we also need a termination network $T_A$ which provides a probability of termination $\tau^{(t)}$ after applying a sigmoid activation function 
$$\tau^{(t)}=\sigma(T_A(\bf{H}, \bar {\bf{H}^{(t)})})$$
with 
$$\bar{\bf{H}^{(t)}}=\frac{1}{|V|}\sum_{i \in V} \vec{h}^{(t)}_i$$
The points from 1 to 4 will be repeated  if the algorithm hasn't terminated e.g. $\tau^{(t)}>0.5$ and parts of $\vec{y}_i^{(t)}$ will be potentially reused for $\vec{x}_i^{(t+1)}$ 
## the models
In the proposed experiments all the algorithm dependent networks have been used as linear projections and the representational capacity has ben concentrated on the processor network $P$, in particular we emply a GNN as $P$ and we will compare graph attention netowrks with message passing neural networks 
![[Pasted image 20240610164637.png]]
In this case on the left we have the GaT where $a$ is the attention mechanism producing scalar coefficents with a learnable projection matrix $\bf W$ 
$M$ and $U$  are are neural network producing vector messages and $\bigoplus$ is an elementwise aggregation operator e.g. max,sum or avgand M and U are used as linear projections. Also P is agnostic to algorithms so it can esecute several algorithms simultaneously.

# experimental setup

The learner has been provided with a variety of input graph structure types in particular they are of seven types:
- ladder
- 2d grid
- trees 
- erdos-renyi with edge probability $\min(\frac{log_2 |V|}{|V|},0.5)$ 
- barabasi-albert attaching either four or five edges to every incoming node
- 4-community graphs generating four disjoint erdos renyi graphs with edge probability 0.7 and interconnecting nodes with edge probability 0.01
- 4-caveman with each intra clique edge removed and prob 0.7 by inserting shortcut edges between cliques
additionally insert a self-edge for every node in the graphs to support easier retention of self-information through message passing. Every edge has a real valued weight drawn uniformaly from the range $[0.2,1]$  which is used to ensure uniqueness of the recovered solution. Corner case are ignored.
The training was done with 100 training and 5 validation graphs for each category of20 nodes each. for testing a 5 additional graphs of 20,50,100 nodes are generated per category.
## the algorithms
## parallel algorithms
First we consider parallel algorithms, BFS for reachability and bellman ford for shortest path here is their formulation as graphs 
![[Pasted image 20240610173628.png]]
![[Pasted image 20240610173651.png]]
in BFS we don't need additional information so $y_i^{(t)}=x_i^{(t+1)}$ , in the bellman ford instead we might have predecessor node $p_i^{(t)}$ in the shortest path which is crucial in the output and is defined as 
![[Pasted image 20240610174219.png]]
and we get that $\vec{y}_i^{(t)}= p_i^{(t)}||\vec{x}_i^{(t+1)}$ and instead of using $+\infty$ we just set it to the longest shortest path+1 for numerical stability reasons.
These algorithms can be learned simultenously by concatenating the relevant $\vec{x}_i^{(t)}$ and $\vec{y}_i^{(t)}$ values for them.
## sequential algorithms
In this case we have a single iteration that focuses on a single node at a time, here we tryu the Prim algorithm for minimum spanning trees
![[Pasted image 20240610180501.png]]
![[Pasted image 20240610180515.png]]
