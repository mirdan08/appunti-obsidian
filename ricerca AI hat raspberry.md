# Ricerca su AI Hat di raspberry
Questo documento serve a mostrare le possibilità di utilizzo del rpi AI Hat.

Prima di cominciare ci sono delle cose di cui tenere conto:
- è possibile applicare la "quantizazione" ai modelli gia presenti, questa procedura sostanzialmente comprime i modelli passando i valori da 32 bit a 16/8 bit consentedo un grosso risparmio in tempo e memoria al prezzo di una leggera perdita di precisone, in base al modello anche <<1% di accuracy o misure equivalenti. Tale procedura consente di utilizzare modelli che a cose normali non sarebbero eseguibli su un AI hat.
- é anche possibile "distillare" un modello dove da una architettura più piccola si può imitare una più grande ed avrebbe effetto simile a quello della quantizzazione ovvero ridurre costo in termini di memoria e tempo. Applicandola si ottiene risultati simili  e modelli grandi potrebbero beneficiare.

l'AI hat ha due versioni 13 TOPS e 26 TOPS, semplicemente una è molto più potente dell'altra con più operazioni,misurate in Trillion of Operations Per Second, consentendo l'esecuzione più o meno veloce ma non cambia la memoria impiegata per fare inferenza quindi non è una questione di dimensioni.


il nostro dominio di applicazione è la computer vision pertanto analizzeremo solo le applicazioni utili in questo senso.

Per quanto riguarda l'analisi della pelle esistono vari modelli utili:
- per classificare la cellulite : esiste MobileNetV2 è un modello preallenato, significa che non è pensato apposta ma con un dataset si può finetunare sulla classificazione della pelle. In alternativa anche ResNet, ha lo stesso scopo ma con un'architettura diversa in base ai dati e può essere conveniente usare questa.
- Per quanto riguarda la segmentazione delle immagini, Unet consente l'uso ma con quantizzazione quindi non può essere modificato ulteriormente per i nostri scopi ma potrebbe migliorare tantissimo il riconoscimento di una persona. In questo caso possiamo usare alternativamente FastSCNN con quantizzazione.

L'unico limite è la dispoibilita limitata di dati ma è possibile provare a ricavarli tramite tecniche di datamining unica problematica sarebbe l'etichettatura ma con alcuni metodi è possibile utilizzare una quantità limitata di etichette ed avere buoni risultati, è da approfondire questa ipotesi.

Apparte l'uso di modelli gia pronti è possibili compilare modellli nuovi ad hoc e fare interagire il modulo con un'applicazione sulla CPU, quidi possiamo eseguire in parallelo conti molto pesanti oltre a modelli gia pronti, ciò milgiorerebbe moltissimo le prestazioni  e consente l'uso di modelli molto pesanti in tempi ragionevoli.

Esistono anche modelli multi modali cioè in grado di comprendere più tipi di input,esempio : generare testo da una immagine o viceversa ma sono molto pesanti e per il momento poco significativi.

Altre possibilità interessanti sono l'uso di LLM da capacità limitate ma in grado di leggere i dati del paziente e rispondere, anche questi leggere ma in questo senso esistono molti modelli ed è un campo molto esplorato con librerie gia create appositamente per cose di questo tipo, da apprfondire ma molto probabile che dia buoni risultati.

In sostanza l'AIhat ha un buon potenziale ma deve essere testato adeguatamente.
