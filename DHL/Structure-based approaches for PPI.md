This document serves as an analysis for structure-based PPIs.
Main approaches are geared toward ML and DL, while also giving insifghts into functional sites like binding and catalytic residues.

We need a lot of negative samples and high resolution structural data si scarce.

To study PPi data can be used on a broader scale, and mapped into interaction networks that are based on physical or functional associations. Despite its incompleteness the PPi studies elp understand the link between structure and function. A lot of methods to analyze such approaches as been developed. 
A lot of these are from the biophysical prospective, which we don't care for, tht while being the most analytically exhaustive also proves as a problem since most of these methods are innately limited e.g. dificult to reproduce and/or difficult to detect in the first place.

In the early days the interatcton ws that two proteins interact if they come from the interacting domains. The Domain-domain intraction prediction was initially based on statistical technqieus like MLE and association methods, but ML seems to be the best one as of today like RM and graphtheory proves to be more useful with very big advancement in recent years for ai based algorithms.

First apporaches where on physicochemical properties using SVMs or other ML algorithms. Other take advtange of the primary protein sequence using a multiscale local descriptor for feature representaton of  diferent lengths of amino acid sequences and an RF classifier.
Recent methods use :
- Gradient Boostin Decision Tree
- Ensemble Extrem Learning Machine
- Principal Component Analysis (for feature reduction not as an algorithm)
Deep learning methods also are very useful,:
- Autocovariance and conjoint triad can do differential feature extraction from protin sequences and uses a stacked autoencoder with a softmax classifier
- CNNs are widely used for both extraction of sequcne dervied features and prediction task themselves.
These methods cannot compete with structural based ones in accurcy even though they have very big sets of data.

## PPI data
Over the years lots of databases have been built, curently vrious are recognized in the literature like DIP,MINT, Biogrid, IntAct and STRING. These DBs PPIs can be used for trainingn and preicto in PPi prediction to asses accuracy of findings and the available protein interactions have been experimentlly verified.

For our purposes we also need "negative PPI" datasets containing PPi that do not interact and Negatome is one of them with 3D protein complexes but results are limited,.
Negative PPi are often generated using subcellular localization or random sampling but we could introduce biases overestimate prediction accuracy or result in unblanced datasets especially when styduing specific biological contexts.

## Biological structure
PPI data is in the ofrm of features, for strucutre based approaches the the key feature is the protein strucutre itself that can be retrieved from ProteinDataBank with experimentally supported 3D structures of proteins orthe AlphaFold DB. From uniprot other supplementary features can be retrived rangin from:
- protein sequence
- post translatonal modifications events
- expression subcellular locaton variants
- implicatons in diseases
- family and domain information 
- Go annotations
![[Pasted image 20260414152733.png]]
![[Pasted image 20260414152751.png]]
![[Pasted image 20260414152803.png]]

## PPI prediction methods
The PPi methods fror structure based use a lot of ML and DL techniques, below we have them framed in  a table for convenience.
![[Pasted image 20260414152926.png]]
![[Pasted image 20260414152940.png]]
All these are computation efficient for medium size datasets.

For each one we will have a thorough analysis
### SVMs & RBF kernels
SVM is trained to tell apart interacting and non-interacting surface patches using six surface properties:
- surface shape: defined by a shape index and a curvedness
- conservation: its a score based on sequnce homology generated using Scorecons program.
- electrostatic potential: comuting using Delphi sotfware which incorporates Amber atomic changes nad gridbaes extrapolation for protein surface
- hydrophobicity: calculated using Facuhere and Pliska's scale for hydrophobicity
- residue interafce propensity: indicates wehtere residues occur more frequently at the interface
- solvent accessible surface area (SASA): calculated usinga MSMS program.

The dataset contains 180 manually curated proteins repsenting transient and obligate protein interactions filtered for natural and stable dimers. Negative samples of equal size as the training dataset and random patches  of nonintracting surface regons were also chosen.
Validation is conducted with **leave one out cross validation** to evaluate model stability.
### 2stage SVM (NOXclass)
This uses again SVMs but in two stages, it distringuishes betweeen obligat interaction and non -obligate interactions and crystal packing contacts, its a different kind of  PPI classification.
The two stage SVM first classifies  the interaction as biologically relevant  or not iff SVM1 binary classifier decides it is relevant and if it is then SVM2 decides if it is obligate or not.
Various features are used:
- Interface Area (IA)
- ratio interface area/protein surface area (IAR)
- amini acid compositionof the interface (AAC)
- corelaton between AAC of interface and protein surface (COR)
- gap volume index (GVI) quantifies shape complementarity  by normalizingthegap volume between interacting surface of protomers againts their surface area
- conservation score of the interface (CS) calcualte using ConSurf 
Cross validation was conductedto assess the feature selection. IA,IAR and AAC are the best ones and ahicceve up to 91.8% when used others seem to add noise in the classifier
### Random Forest
Combines molecular modeling sutrcutrla bioinformatics ML and funcitonal annotation filters. 

Docking and docking refiments algorithms are used to predict the structure of hte proteins in every protein pair. Followed by hte identification of their binding site and finally dimer prediction and its refinement, the ML algorithm of choic is an RF classifier that is used to predictthorugh assginment of a probability score wheter a dimer is a true PPI.

The used features fro mthe RF classifier of ths study are retrieved from FiberDock a backbone refinment algorihtm. After the prediction protein pairs that were classifed as interacting were passed via a Go term filter to ensure they shared cellular locations and participated in the same biological process but had different molecular functions. The pipeline use a BM1905 dataset previously with more or less 20000 samples and a RandomForest evaluated on a 10 fold cross validation with metrics like ROC, AUC and FPR.


RF is also used in PPI prediction using a lot of single amino acid features ranging from enire protein propertiesm feature of the primary and secondary structure of the proetin as well as a 3D structuralfeatures was and an intricate features selection module.

The evoluationary conservation feature was quantified using PSSMs generated bu PSI-BLAST, repsetned the likelihood of eachreside to be conserved instead of mutatiin to each of the 20 amino acids. Amino acid proterites are represented using five numerical patterns retrived from the AAIndex DB ,refleting polarity secondary structure molecular volume codon dieristu and elctrocstatic charge. protein disordered regions crucial fro biological function were analyzed using VSL2 to calculat disorder socres for each amno acid.
Secondary strucutral features inlcuding secondar strctures and sovlent accessibility were predictd using SSpro4. The 3dstrucutral feaures extraced from PDB database were used n the stdy were the protrusion index and depth index are predicted by PSAIA as well as SASA with a molecular surface area and surface curvature compute from SurfRace.

The minimum redundancy maximal relevnace method was employed to rank features by evaluatin their relevance to the targt and their redundancy wit hother features the result was then used in the Incremental Feature selection. to determine the optimal feature set.

The dataset was constructed using 3did database using interactions from known structures

### Bayesian Networks
PrePPI methods uses strucutral and non-structural featurs to model PPi across yeast and human proteomes.

Structurs are soruced from PDB  for high equence identity methces or dervied from ModBase and SkyBase homology models in cases where proteins did not have epxiermentally verifified structures. result: 8582 PDb structures and 30,912 models for human proteins. Structural neighbors were identfied iwth the Ska alignment tool performingstrucural alignment depending only on the geometric shape.
We use five modeling features:
- Structural Similarity(SIM)
- Size conseved interacting pairs (SIZ)
- Coverage of interacting pairs (COV)
The previous three show whetther the interce of a template is present in the model
- overlap score (OS)
- overlaped predicted intrefacial resides (OL)
Evaluate if the residues i the model interface have compatible characteristics with rsidues that mediate recognized PPis.
### Artificial Neural Networks
The prediction is whether each surface residue is in contact with the protein a 11-residue window that includes the residue of interest and its spatial neighbors. Features extracted for this prediction include evolutioary conervationprofiles dervived from sequence alginments inthe HSSP databse and solvent accssibility calculate by a DSSP program with rsides coded into 20-D verctors representing their sequence convservation frequencies. with a three fold corss validaton process for validation.
#### CNNs
SpatalPPi uses protein complexes predicted by AlphaFold Multimer and classifies their interaction with a 3D CNN using rigorous curation processes were used to address class imbalance and eliminat redundancy as a considerable overlap exists between protein pair in both data and soruces. the model was trained on 600 positivi PPi pairs from thebiogrid database and 600 non interactive pairs curated from negatome 2.0 , each pair ws put into AlphaFold multimer to predict the individual strucures optimizing the resulting coplex.

Fro mthe predicted protins we convert the structure into a 3D tensor to representsptialaotmic arrangements three encoding strategies where used: one-hot.volume and distance encoding. The encoded features. A 3D CNN was employed followed by a F layer classifiying it into interacting and non-interacting.
Two main architectures:
- Residual Blocks which is a ResNet adapated
- DenseBlocks which is a DenseNet

both used 3d pooling and calcualte a 1D feaure vector, validation ofthe SpatialPPi model was done across a 5-fold cCV coupled iwth clustering based subset selection stratgy to ensure  that similar proteins were assigned  to the same subset multiple combinations of the CN arhictectures and tensorizatino methods were initially evaluated  and DenseNet-based 3D CNN using a distance tensorization has emerged as the most accurate with a mean of 0.81, 0.89 of AUC and 0.83 of precsion and recall of 0.79 with low average and standard deviation of accurayc across folds highlithing tability.
#### Graph neural networks

- Struct2Graph: The struct2graph model employs a raph attention network (GAT) for PPI prediction using only a graph representation of 3D structural data of proteins and not specific strucutral properties such as SASA and hydrophobicity that have been employed by other reserchers. with a dataset made of 120,000 circa protein pairs with available structurs from PBD databse.
	- positive pairs are obtained from concordant mathces between physical PPis of STRING and IntaACT after excluding co-localized protins
	- negtive samples are considered ot be pairs with no interacton evidence and not part of any interaction in STRING nor IntAct
	the ramework converts protein structures  into raph where maino acids are represented as nodes connect by edges if their spatial proximity is within 9.5 and uses 1 hop neighborhoods to capture local information processed into a GraphConvolutionalNetwork, the resulting embedding are aggregated using mutual attentionmechanism and the final classfication is done using a feedforward neural network. Model validation is done using a 5fold cross validation on balanced and unbalanced datasets varyig positive to negative ratios. Result are all above 90% across all scenarios and metrics even the most imbalanced ones
- We can predict PPIs combining GNNs with language models to create enhanced protein representations thetraiing datast includes 16,200 and 2847 positive PPis from saccharomyces from the HRPD databse and DIP. negative samples where consstructed by randomly pairing proteins from the positiv datast that are localied indifferent subcellular locations and non PPis from the negative DB negatome. feature extrcation happened with a LSTM layer and BERT framework in this case both GAT and GCN are evaluated to generat protein embeddings which are conacnated and a fully connect network is used in the end.
- MAPE-PPI is a computational technique, it extracts microenvironemnt-aware embeddings based on the protein structure capture by a fine-grainde codebook thorugh a variant of quantized variational autoencoders (VQ-VAE).
	- Each chemical residue is encoded into a discrete code containinginformation releevant to each residue's surrounding chemistry and geometry the result embeding is a node feature n a PI graph mde up of all the protein whose interactions are to bepredicted,i.e. the edge hidden types, and he PPis of the rainig dataset itreaction types are of seven kinds
		1. activation
		2. binding
		3. catalysis
		4. expression
		5. inhibition
		6. post-ranslational modifcation
		7. reaction
- Graph Isomorphism Networks, can be used on tope of the extracted embeddings from MAPE-PPI
The splitting of the DB is conducted in three methods: Random split, BFS and DFS.
- The hierrchical Graph Neural Netork for PPI (HIGH-PPI) is an advanced framework for multi-ype PPi prediction integraite hierarchical graph structure nad an explainer moduler to get key residues involved in protein interactions, this encapsules bot resisdue-levl and prtoein leve intractione enablingthe model to tlearnglobal features and fine grained features, it uses a dual pov approach:
	- bottom view uses a GCN to extract residue-evel features
	- the top view uses a GIN to caputre network level PPi properties 
The embeddings from the bottom iew are used as inputs for the PPigraph processed by the top view and the reuslting protein embeddings are subsequently classfied using a MLP, also disposes of a XAI component specifcally GNNexplainer toidentify functionally critcitcal reisdues suc as bindg or catalytic sites it as superior perofrmanes and generalizes better to OOD scenarios w.r.t. other methods. tested with 5 folds CV  using area under precisionrecal cureve (AUPR)
### Clustering
The AlphaBridge ramework is an advancement w.r.t. AlphaFold3 it uses predict local-distance difference test, pairwise aligned error nad predicted distance error. THese metrics are integrated into graph-base clusergin approaches enbalbing precise identification and analysis of interacioninterfaces of macromlecular complexes with PPi and proteinnucleic interactions. using sopysicated chord diagrams data is visualized and sequcne convserveation scors enable intrpretability. The framework uses metrics comping from AlphaFold3 outputs with preprocesings centered on construcitg the predict mergd confidence matric a fusione of PAE a pLDDT data refined iwth communitu clustering algorithms and multidimensional image processing techniques. Empirical evaluation involved AlphaFold3-predicted models including cases such as human mismathc repari proteins interacting withnucleic acidds. validaton focues on robusntess of predictions and active visuliazation to assessconfidence level.