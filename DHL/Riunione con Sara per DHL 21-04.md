Dubbione per mail a martina

Scegliamo i DB
- Intact: molto ocmplete , usa uniprot id quando viene fatta la query da anche gli altri gene symbol alias per quelli usato, tipo di interazione, metodo per rilevare l'interazione
- Biogrid:usa uniprot ma mostra risultati con gene symbol, bello ma è troppo ambiguo
- String: bisogna chiedere se alcune interazioni sono utilizzabili perchè usa gene symbol ma anche ensembl_id(protein)
	- dobbiamo rimappare  ensembl_id(protein) verso uniprot per usare SPRINT che richiede espressamente gli uniprot ID, non sappiamo come usare lo score
- Interactome: Come string ma ha molte meno informazioni
- Negatome: Solo PPI negative, id usato ed è linkato a PDB ma non per tutti facilemente reperibile, circa 6500 negative non filtrate per umani
- ProteinDataBank: Non pratico per le PPI

Selezionati: Negatome, Intact, String