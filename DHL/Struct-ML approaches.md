Questa analisi serve a mettere insieme la proposta per l'approccio struct-based su metodi ML da utilizzare nel progetto di DHL.

Lo struct based è molto vario ed usa molte propietà diverse provenienti dalla struttura delle proteine
- eventi post traslazione
- espressioni delle variazioni di locazioni sub-cellulari
- implicazioni nelle malattie(in quali sono rilevate)
- famiglia della proteina e informazioni dei domini dove viene usata
- GO terms vari con la quale sono annotati
- nel caso degli ibridi usano anche informazioni provenienti dalla sequenza degli amminoacidi.
Nel caso dell'ML ci sono varie famiglie di modelli con i loro "elementi di punta" per ognuna di esse, essendo vaste ci si concentra principalmente su ciò che in letteratura ha già dato buoni risultati come nel caso del paper riportato.

I databse utilizzati per le PPI sono Negatome per i non interagenti ma hanno poche proteine, si può generare sample negativi in altro modo l'informazione diventa troppo rumorosa. Per le effettive PPI DIP,MINT e BioGrid sono presenti.

Per le strutture 3D ProteinDataBank o L'AlphaFold database sono usati per ricavere le informazioni.

Uniprot fornisce invece le altre feature strutturali spiegate nell'elenco puntato precedente. 

## SVM con kernel RBF

Sono state usate per per i siti binding della PPi in questione ed anche per i tipi di interazione con informazioni ricavate tramite programmi esterni
- in un approccio hanno usato la cross validation per prendere informazioni quali forma della superfice,potenziale elettro statico,idrofobicità e molte altre propietà provenienti dalla superficie della proteina ricavati tramite programmi esterni, risultati fino al 76 % con una leave-one- out-cross-validation(estremamente costosa in termini di tempo ma precisa quando si hanno pochissimi sample), hanno usato 180 sample che sono pochi.
- Altro uso delle feature della parte superficiale della proteina ricavate tramite programmi esterni e risultati fino al 80% in varie metriche con recall,accuracy etc.etc. con selezione feature accentrata sulle proprietà come dimensione area dove si trova proteina e la composizione di amminoacidi, hanno raggiunto circa il 90%> su tutte le metriche con 243 PPi.
In generale hanno buoni risultati ricavando più informazioni possibili sulle proprietà fisiche della proteina in base a ciò che si può trovare sui DB disponibili, potrebbero essere utili con una feature extraction fatta prima e richiedono SHAP o altri tool di XAI.

## CNN con DenseNet/SpatialPPI

Hanno caratterizzato la proteina utilizzando informazioni ricavate da AlphaFold Multimer poi passato al modello con vari metodi per la codifica dell'output in particolare l'architettura DenseNet sembra essere andata meglio con risultati fino all'80% in varie metriche come prima fra qui AUC,ROC ,accuracy recall etc.etc. con 600 PPI positive e 600 PPI negative.
## Decision Tree con RF e GDBT
hanno usato sia feature delle prorpeità fisiche che sulla struttura 3D della proteina stessa quindi è un approccio misto, usano sia proprietà strutturali 3 dimensionali  con molta selezione delle features con DB sui 10.000 samples circa raggiungo risultati al massimo del 70%


## GNN con Struct2Graph

Le GNN sono numerose ma Struct2Graph è stata trainata e pensata apposta perriceere struttura 3D  ad alta qualità, è estremamente efficace arrivando fino al 90% in molte metriche anche in  casi altamente sbilanciati con circ 117.000 samples ma dati di alta qualità e ne richiede tanti quindi dipende da quanti ne abbiamo noi.

## Bayesian networks

Usa altre features strutturali ma ha risultati non buoni oltre ad essere usato su proteine non-umane ed umane quindi non sappiamo come si comporterebbe solo sul proteoma umano, molto probabilmente non daranno buoni risultati con circa 30.000 proteine non-umane (dal lievito) e circa 30.000 umane.