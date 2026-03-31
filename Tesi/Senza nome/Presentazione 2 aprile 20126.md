# RIPPLE++

- stream online processing on large scale dynamical graphs
- Novelty: incremental model that uses GNNs semantics to incrementally propagate updates with additions/deletions of edges/vertexes and vertexes features updates.
- Breve introduzione alle GNN
	1. fase di inferenza
		- inferenza su grafi dinamici
		- determinismo vs non-determinismo implicati dalla funzione di aggregazione
	2. distribuzione/parallelizzazione
		- vertex centric
		- layer centric
			- memoria occupata 
-  RIPPLE++
	1. caratteristiche
		1. aggregatori monotonici e cumulativi ed aggiunta GAT
		2. routing locally aware 
	2. propagazione layer-wise di modifiche al grafo
-  propagazione incrementale
	1. BSP e superstep per GNN
	2. messaggi propagati incrementalmente
		1. GAT extension
		2. monotonic aggregation functions
	3. analytical model
- RIPPLE++ inference model
	- server worker exectuion model
	- locality aware routing
	- request batching
- breve discussione dei risultati
	- osservazioni sulla topologia per collergarsi a GSM
# GSM
- spiegazione problematiche gestione GPU con challgenges da 1 a 4.
- pipeline di inferenza in GPU
- osservazioni 1,2,3 e 4
- spiega il parameter optimizer
	- lighweight request scheduler
- discussione risultati