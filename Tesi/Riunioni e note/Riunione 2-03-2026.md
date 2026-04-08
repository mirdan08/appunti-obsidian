## Paper osservati
- RIPPLE++ propone un framework generico che comprende molte funzioni di attivazione e architetture per GNN. Fa inferenza esatta in tempo reale con un'architettura distribuita che sfrutta la semantica delle GNN per evitare il continuo ricalcolo degli embeddings/attivazioni latenti.
- GSM propone un sistema semi-automatizzato per gestire workload dinamici usando una GPU, bilanciando la memoria usata per caching e computazione andando  ad ottimizzare l'overhead di comunicazione fra cpu e gpu ed il tempo di calcolo sempre con inferenza esatta ed in tempo reale.
- Helios, non utile per i nostri scopi ma comunque interessante, propone un'architettura per il pre-sampling continuo su un grafo dinamico con un'approssimazione sul vicinato di ogni nodo per questioni di efficienza che sfrutta il caching e lavora molto sulla fase di sampling per minimizzare la latenza.
## Osservazione su papers ed idee
- Decentralizzare la parte di "routing" del router/load balancer in RIPPLE++
	- assumi un datacenter quindi la topologia della connessioni non è un problema e concentrati solo sulla decentralizzazione/il routing del workload.
- valuta aggiunta di gestore dinamico per la topologia dinamica, in particolare presta attenzione ai burst di calcoli/memoria che in GSM vengono gestiti ma in RIPPLE++ non sono considerati.
- GSM è statico e dipende dalla coppia modello-dataset, il che è molto restringente, vedi se si può fare qualcosa al riguardo e renderlo più flessibile.
- valuta uso GPU, sembrano essere più dannose che altro soprattutto in caso di workloads dinamici e misti ma forse sono utilizzate come pre-training prima dell'inferenza e potrebbero risultare comunque utili.
- Nella fase sperimentale di RIPPLE++ non viene considerato il caso avversario di una topologia che cambia drasticamente, cioè fra un worker e l'altro non vengono mai spostati i nodi e la variazione effettiva della topologia nel tempo non viene presa in considerazione. 
  Potrebbe essere dunque interessante pensare di aggiungere:
	- valutazione sul caso avversario appena spiegato per la fase sperimentale della tesi.
	- possibile proposta per riassestare la distribuzione della topologia e fare ridistribuire fra gli worker i nodi/archi del grafo durante la fase di inferenza quando questa diventa diventa troppo inefficiente a causa della topologia.
	- in generale lavorare comunque sulla gestione di una topologia dinamica.
- Sulle tecnologie effettive c'è molta varietà per ora la più efficace sembra essere MPI con python e l'uso del motore di inferenza DGL come in RIPPLE++, da valutare quando il lavoro è più impostato.
## Proposte per tesi
Un'architettura per inferenza esatta in tempo reale simile a RIPPLE++, che sfrutti quantomeno la semantica delle GNN per distribuire ed efficientare i calcoli, con un load balancer decentralizzato che sappia gestire grafi con topologie dinamiche per la fase di routing e che ridistribuisca il grafo partizionato quando la topologia accumulata è troppo sbilanciata.
Da valutare sono:
- impiego di GPU.
- tecnologie effettive per l'implementazione vista la varietà di approcci.