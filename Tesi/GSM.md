In GNN some applications require exact online inference, meaning we process al neighbors and not a processed subset for accuracy and determinism. In this context head of line blocking and hidhly voltaile memory usage complite optimization in CPU-GPU communication and computation. 
GSM proposes an approach to this issue using three main components:
1. A parameter tuner, which optimiz CPU-GPU communication overhead and computation.
2. A degree based scheduler, which groups requests with simila in-degrees into batches giving priority to requests with shorter processing times.
3. A batch divider, which partitions a subgraph to achieve optimal subgraph size and optimize latency.
We assume GNNs are known, we focus on labeling tasks with *target vertices* being our element of analysis. 

# Introduction
We start with a few observations on which GSM is based:
1. The in-degree sum of a batch can serve as a simple predictor for batch processing time
2. Due to the neighborhood explodig depending on the target, memory usage is highly volatile.
3. Across datasets and models GNN workloads differ dramatically and optimal prams lke bathc size,cache size and batch subgraph size so we need a way to select them. 
4. For GPU this is a problem since cache vries it's size to make room for activations and parameters, if activations size explode due to exploding neighborhoods cache shrinks complicating caching and increasing comm. overhead between CPU and GPU
![[Pasted image 20260330140452.png]]
## Challenges in online inference exact inference for GNNs
All these observations explain long and unstable inference latency, we can formalize three challenges to adress these issues:
1. Efficiently predict processing time for an inference request: Scheduling based on processing time is the best way to avoid this, on GNNs it's difficult and is sually done in batches.
	1. Extract L-hop neighborhood subgraph for target vertices, stored in CPU.
	2. preprocess in GPU friendly ofrmat 
	3. transfer to GPU for the forwrd pass
	Here the neighborhood subgraph can be individually to estimat it's cost but this is very slow , very prohibitive for our purposes, and overlapping neighborhoods make it impossible to isolate each target vertex's contribution.
2. GPU efficient usage: High memory usage restrict space for cachng and increse CPU-GPU data transfers, suppose to increment batches this will mean more activation memory and less space for cache requiring more communication i.e. more comm. overhead. Also skewed degree distribution makis this highly variable, we must conserve memory to avoid OOM errors limiting cache and batch size even more.
3. Efficient parameter optimization: varying all systems parameters requires balacning computation and communication costs but such parameters interact in very complex ways. 
	Larger batch subgraphs improve GPU efficiency but when workloads are communication bound larger caches instead benefit from smaller batches to allow more caching. There is no global optimal configuration.

These challenges are adressed by GSM a **G**NN inference serving systems with efficient **S**cheduling and **M**emory management, composed of three components:
1. Lightweight request scheduler, from obs.2 we prioritize lower in-degrees request to achieve optimal in-degree sum per batch, balacning computation and comm.
2. Batch divider, from obs.4 we understanthat few batches can pea memory usage so we use a batch divider partioning oversized graphs when GPU memory is not sufficient, memory usage is stabilized and reduced to enable larer caches and batch sizes.
3. Paramer tuner, the parameter tuner adapth in-degree sum and batch subgraph thresholds to each workoad, it uses two 1-D searches for the thresholds.
## Exact online inference for GPUs
In the context of exact online inference a rnning service receives queries for individual vertices called requests. Each requirethe output of a target vertex from a L-layer pretrained GNN,mltiple requests usually get batched to shared GPU resources during inference using their combined *L-hop neighborhood*.

Large graphs cannot reside in GPU memory, both strucutre and featuresare stored in CPU memory, GPU executes inference in three stages:
1. Sampling & layout transformation: For each batch, the system samples L-hop neighborhood of all vertices to form a subgraph, first *layout transformaton* is done meaning data is remapped into contiguous GPU memory adresses for efficient and kernel executon, sampling is done layer by layer.
2. Fetching: the preprocessed subgraph structure and its vertex features are transferred to GPU memory, frequently accessed features are cached
3. Forward operation: The GPU performs layer-wise message passing and activation over the subgraph to produce outputs for all target vertices.
$1 \rightarrow 2 \rightarrow 3$ is the classical  GPU inference pipeline:
- usually sampling and fetching are communication heavy.
- layout transformation and forward computation dominate computation.
### Experiments
Used models
- GraphSAGE with 256 hidden size.
- 3 layer GAT model with also 256 of hidden dimension. 
Used datasets:
- Papers100M 
- Friendster
Computer
- NVIDIA A100 GPU (40GB)
- AMD EPYC 7742CPU (8 core, 300GB RAM)
-  GPU-CPU interconnect with 22GB/s bandwidth
## Analyzing batch processing time
Batch processing time is highly variable, a naive scheduling strategy would precompute L-hop neighborhood size, sort requests by size and for msequential bathces.
![[Pasted image 20260330151037.png]]
Here we see how batch processing and comm.overhead are persent across different models and graphs.  In the figure below we can see that batch processing time is correlated with the in-degre sum of target vertices

![[Pasted image 20260330153056.png]]

We can observe that batch cost estimation with in-degree sum is an efficient alternative, the batch processing time is domposed in the *sampling*, *fetch* and *forward* and analyze batch processing time through this elements in the image below
![[Pasted image 20260330153342.png]]

We can see that larger batches reduce sampling and forward time but increase fetch time as cache limitations require to pass more data in and out of the GPU . Optimal parameters vary across dataset and models to minimize average latency.

GPU memory limits optimal batch and cache sizes, when cachng is removed we can see memory usage across bach size in the figure below.

![[Pasted image 20260330154040.png]]

we can see very extended whiskers for each blox plot demonstrating high variation in memory occupation. We also examine form the same experiment the batch memory usage correlatd to batch subgraph size

![[Pasted image 20260330154203.png]]

Batch subgraph size and memory usage are almost correlated, we the size grows so does the memory in an almost linear manner for all models.

This demonsrates the highly memory volatility.
# System design

This is the component we will add to our scheduler.

![[Pasted image 20260330154432.png]]

This new component will
- Use a lightweight scheduler using in-degree batchprocessing estimation.
- Apply a batch divider to mitigate GPU memory and prevent OOM errors.
- Tune parameters select the optimal in-degree sum and batch subgraph size thresholds for each workload.
In the image
- the request dispatcher, i.e. the scheduler, uses in-degree estimation to group into balanced batches.
- the batch divider keeps each batch graph under the memory threshold by splittinf oversized subgrahs into smaller subgraphs to be processed sequentially.
- The parameters tuner,i.e. parameter optimizer, runs offline.
### Lightweight request scheduling
no fixed batch size is used, instead to reduce latency we scheduler first requests with shorter expectd procesing times using the in-degree sum and group them into batches with the total in-degree sum reaching the optimal threshold. The objective of the scheduler is:
1. preventing head of line blocking, with a FIFO policy you would have long latency blocking other low latency requests.
2. Efficiency: request should be bathed into appropriate sizes to prevent OOM and high latency.

the scheduler predicts each request's cost 
- from its in-degree sorts the queue by ascending order
- adds requests sequentially until the cuulative in-degree sum threhshold $T_d$ is exceeded 
$T_d$ is determined by the parameter tuner balancing computation and communication, too small values under use GPU and too large risk excessive memory use with the consequences we already discussed.
### Dividing the batches
Feature fetchng is the most costly operation portion of the infernce in GPUs, a bigger cache sizes reduces the need for fetching but feasible cache sizes reduces this to maintain extra space and avoid OOM errors the GPU remains largely unused during inference. Batch division should flatten peaks. and increase available cache size while reducing communication load.

The batch divider reduces and stabilizes memory usage by dividing the batch subgraph into smaller subgraphs, in general bathc ubgraph size is a key parametrs to keep under control.
$T_d$ serves as a memory control but only cares for the first-hop, in deeper hops neighborhood explosion is still a problem and leads to OOM or use too much memory in general. We could reduce cache size, this will work but this incrments comunication overhead and decreases cache size i.e. slows down inference. In thi context bathc division is a surefire safeguard against OOM errors when memory spikes happen. We can reduce it byt limiting the number of vertices in each layer of the batch subgraph, we set a threshol for the maximum number of vertices  at each layer named $T_s$ . if a layer subgraph execced $T_s$ the sampler divides into smaller subgraphs oless than $T_s$ vertices. $T_s$ determines the inference memory usage, per-request efficiency and cache size and is tuned automatically. 

At each layer the GPU fetches the neighborhood accoding to $T_s$ performslayout transformation for each subbatch and allocates the remaining memory for caching. The inference engine processes subbatches speartely and concatenates thir representations before continuning the forward pass as normal.
### Parameters tuning
Offline the tuner determines $T_d$ and $T_s$ thresholds, we analyze the tradeoffs between these parameters and how they affect performances.
$$
t_l^i = t_q^i+t_p^{B(i)} = t_q^i+(t_s^{B(i)}+t_c^{B(i)}+t_f^{B(i)})
$$
where:
- $t_l^i$ : queueing time for requests for request $i$
- $t_p^{B(i)}$ is the processing time for the batch in request $i$
	- $t_s^{B(i)}$: preprocessing time
	- $t_c^{B(i)}$: feature fetching time
	- $t_f^{B(i)}$: forward time

Increasing $T_d$ makes the batch size largr nad take longer to process due to higher workloads in preprocessing , communication and forward operation. Larger batch sizes also limit cache size with the consequent high communication load.
on the other side larger batch request also more efficient on a per-request basis given fied cache sizes. 

In short n high $T_d$ reduces the queuing time $t_q^i$ , the pros migh outweight the cons resulting in a system the clears faster the queue hence the reduction.

A lower $T_s$ reduces batch processing efficiency on a per request basis and adds a division penalty to the sampling time but allows for larger cache sizes reducing comunication time.

#### Selecting the parameters
An exhausting search to determine $(T_s,T_d)$ is expensive, we rune two separate 1-D searches:
- select various  $T_d$ values empirically
- pick various $(T_s,T_d)$ couples by fixing $T_d$ and running various $T_s$ values, for each $T_d$ note the lowest value 
All obtained pairs are tested with different arrival rates to find the lowest pair. Each sequential processin has a loer computational load which does notsaturat GPU and increases queueing time,but reducing in-degree sum batch processing time is not shortened and in general when compute dominteatesthe cost is higher for lower $T_s$.
