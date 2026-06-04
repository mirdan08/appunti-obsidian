
## Kinds of dynamic graphs
In a temporal graph we have multiple snapshots of the same graph providing historical information and they provide snapshots in th dynamic graph at fixed time intervals.
Representation is of two forms:
- the graph is a sequence of snapshots taken at fixed intervals. It's coarse grained since the order of changes is not preserved between snahost. 
- For timestamps instead each elements has a timestamp  indicating when they were inserted and/or deleted from the graph with finegrained information preserved.
Streaming graphs are comprised of gtraph elements and/or changes to existing elements or in a batch-update fashion. In streaming we only have one version of the graph and the removed elemtns are not stored only the current properties are available and histroical information is lost. This has a smaller memory footprint which is far smaller than that of temporal graphs sine removed elements are not store.

## Dynamic graph processing

The paper explores incremental algorithms, incrementalization is combuersome for memory and requires knowing well how updates are to be propagated .

## Graph partioning

For dynamic graphs the structure changes over time and there might be moments that require the reassignment of graph elements to different partitions. since the structures changes over time it might be nessary to reassign graph elements to different partitions. To provide this without re-partitioning the whole graph according to various heuristics.

Initially we place new elements  to minimize vertex or dege cut, when certain thresholds are met existing elements become candidates for repartitioning.

We can use various kind of features:
- topology related: vertex degree, vertex replication rate, numbner of vertex neighbors in each partition can be used to decide initial element placement.
- Application related properties like the frequency in which quries request certain graph elements and query throughput can be used as a metric to trigger rebalancing of partitions 
- Hardware properties like memory occupation and processing capacity and communication cost can be used when partititoning elements in distributed heterogenous environments.
