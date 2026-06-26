# Regole principali e modalità di consegna

L’avvio del progetto deve essere concordato preventivamente con i docenti. Prima di iniziare il lavoro, il gruppo deve inviare via email al sottoscritto e al dott. Luigi Gargioni un documento sintetico di 1-2 pagine che descriva la proposta progettuale.
La proposta deve essere basata su uno dei temi suggeriti, eventualmente adattato al numero di partecipanti, e deve indicare in modo chiaro:

- obiettivo del progetto;
- componenti software o strumenti che si intende usare;
- scenario o dominio di valutazione;
- risultati attesi e metriche di valutazione;
- suddivisione indicativa del lavoro, nel caso di gruppi da 2 o 3 persone.

# Progetto scelto: 3 - (Pratico) Micromouse con simulatore di maze

Progetto adatto a chi è interessato alla pianificazione e al controllo di un robot mobile in un ambiente discreto e parzialmente osservabile. L’obiettivo è sviluppare un algoritmo per un robot Micromouse in grado di esplorare un labirinto, costruire una rappresentazione interna delle pareti osservate e raggiungere la zona obiettivo minimizzando il costo del percorso. Il progetto deve essere sviluppato utilizzando il simulatore mms (https://github.com/mackorone/mms), che permette di testare algoritmi
di risoluzione del labirinto senza disporre di un robot fisico. Il linguaggio di programmazione può essere scelto dal gruppo, purché sia compatibile con l’interfaccia del simulatore. Attività principali:

- studio del problema Micromouse e dell’interfaccia fornita da mms;
- implementazione di un algoritmo di esplorazione del maze, ad esempio wall following, flood fill, A* incrementale o una strategia equivalente;
- costruzione e aggiornamento di una mappa interna del labirinto a partire dalle osservazioni disponibili;
- gestione dei casi di dead-end, ritorno a celle già visitate e scelta del prossimo obiettivo di esplorazione;
- valutazione su maze diversi, includendo almeno un confronto tra due strategie o due configurazioni dell’algoritmo;
- analisi critica rispetto a:
  - numero di celle visitate;
  - lunghezza del percorso finale;
  - numero di mosse o turni effettuati;
  - tempo di completamento della simulazione;
  - robustezza rispetto a maze di complessità crescente.

# Proposta progetto di Robotica 

**Materiale di riferimento**:
- art. 1 [Micromouse 3D simulator with dynamics capability: a Unity environment approach](articles/s42452-021-04239-7.pdf)
- art. 2 [Optimizing Tremaux Algorithm in Micromouse Using Potential Values](articles/7_Sanjaya_Vol3_No2.pdf)
- video [Virtual Micromouse Maze Mapping and Solving Demonstration](https://www.youtube.com/watch?v=6y4nrnfZ1k0)
- video [Micromouse Maze Simulator - 2018 Japan Halfsize(32x32)](https://www.youtube.com/watch?v=-r8a8aPRYAQ)
- video [Micromouse Maze simulation](https://www.youtube.com/watch?v=0YId4SPJrWo)
- post [Micromouse from scratch](https://medium.com/@minikiraniamayadharmasiri/micromouse-from-scratch-algorithm-maze-traversal-shortest-path-floodfill-741242e8510)
- post [Floodfill Module](https://projects.ieeebruins.com/micromouse/floodfill-module)

## 1. Obiettivo del progetto

Nelle competizioni di micromouse, l'obbiettivo principale è quello di esplorare e mappare parzialmente o completamente il `maze` **online** (**full exploration**) per poi identificare il percorso minimo **offline** per raggiungere il goal dallo stato iniziale entro i limiti di quanto esplorato. Nell'ambito di questo progetto, ci poniamo nello scenario in cui l'obbiettivo dell'agente è quello di raggiungere il goal (di cui si assume conoscere le coordinate), riducendo al minimo la fase di esplorazione del `maze`, prioritizzando l'efficienza ed efficacia nell'individuazione di un percorso, anche sub-ottimo, per raggiungere il goal (evitando **full exploration**).

Nel dettaglio, l'obbiettivo del progetto è quello di implementare e testare differenti strategie di esplorazione del `maze`,ad esempio:  wall-following, flood fill e A*; e confrontarne le prestazioni in termini di efficienza ed efficacia nell'individuazione di un percorso per raggiungere il goal. Poichè l'agente si concentra nell'individuazione di un percorso, la ricerca **offline** del percorso minimo non rientra negli obiettivi dell'agente, ma viene comunque impiegata come indicatore nelle modalità descritte al punto [4](#4-risultati-attesi-e-metriche-di-valutazione).

L'obiettivo è quello di valutare le strategie sopra citate, nei termini di efficacia ed efficienza descritti al punto [4](#4-risultati-attesi-e-metriche-di-valutazione), in relazione a differenti configurazioni del `maze` a complessità crescente e casi critici/sintomatici (ad esempio labirinti con dead-end, loop, isole, ecc.) estratti dalle collezioni di labrinti messi a disposizioni per le competizioni ufficiali di micromouse (ad esempio [`maze-collection`](https://www.tcp4me.com/mmr/mazes/)), come descritto nella sezione [3](#3-scenari-di-valutazione).

Di seguito è riportata una plausibile suddivisione dei contenuti della relazione finale:

**Table of contents (relazione)**:
- Introduzione
- Descrizione del problema di MicroMouse e dell'interfaccia del simulatore `mms` e delle specifiche impiegate (ed eventuali modifiche apportate) nelle simulazioni realizzate
- Descrizione degli algoritmi di esplorazione implementati (wall-following, flood fill, A*) ad alto livello, con dettagli sulla gestione dei casi limite (dead-end, loop, next-point). 
- Descrizione degli scenari di valutazione (`maze` a complessità incrementale, e casi critici/sintomatici) impiegati; e dei criteri e metriche di valutazione adottati.
- Report dei risultati ottenuti, con rappresentazione grafica dei dati raccolti e confronto tra le strategie di esplorazione implementate e analisi critica delle strategie di esplorazione in relazione alle metriche di valutazione definite.
- Conclusioni e possibili sviluppi futuri

## 2. Componenti software

- simulatore di `maze` : [`mms`](https://github.com/mackorone/mms)
- maze file collection : [`maze-collection`](https://www.tcp4me.com/mmr/mazes/)

## 3. Scenari di valutazione

Gli algoritmi saranno testati su un set di labirinti di complessità crescente, selezionati dalla collezione `maze-collection`. Verranno selezionati labirinti con caratteristiche diverse, come ad esempio labirinti con dead-end, loop, isole, ecc., al fine di valutare le prestazioni degli algoritmi in scenari diversi e mettere in evidenza eventuali punti di forza e criticità delle strategie implementate (es., wall-following fallisce in labirinti con isole).

## 4. Risultati attesi e metriche di valutazione

Di seguito sono riportate le metriche di valutazione impiegate per confrontare le strategie di esplorazione: 

- **efficienza di esplorazione**: numero totale di mosse necessarie per raggiungere il goal
- **efficacia di esplorazione**: numero totale di celle distinte visitate per raggiungere il goal (porzione di `maze` esplorata) - percentuale di `maze` esplorato
- **numero totale di celle visitate**: numero totale di celle visitate per raggiungere il goal (porzione di `maze` esplorata), sommando tutte le occorrenze di visita per cella
- **percorso minimo di esplorazione**: lunghezza del percorso minimo individuato offline sulla mappa interna costruita dall'agente
- **percorso minimo del maze**: lunghezza del percorso minimo individuato offline sulla mappa completa del `maze`
- **tempo di completamento della simulazione**: tempo totale impiegato per completare la simulazione (raggiungere il goal da s0)
- **robustezza**: capacità di scalare efficacemente a maze di complessità crescente, e di gestire casi limite come dead-end, loop, isole, ecc. TODO: definire come valutarla

Per ogni simulazione le metriche saranno raccolte in file di log in formato `json` o `csv`, al cui interno saranno riportati i valori consuntivi rispetto a ciascuna, e la rappresentazione interna della mappa costruita dall'agente secondo due matrici `R x C` (con R pari al numero di righe del maze, e C pari al numero di colonne), nella prima, andando a definire per ogni cella esplorata una 4-upla `trbl` con `t, r, b, l = {0,1}`, ed ogni cifra indica la presenza (`1`) o assenza (`0`) di un muro in posizione: `t` -> top, `r` -> right, `b` -> bottom, `l` -> left della rispettiva cella (`None` per le celle non esplorate); mentre la seconda tiene traccia delle occorrenze di visita di ciascuna cella, andando ad assegnare un valore numerico ad ogni cella: `0` per celle non visitate, e `n > 1` per celle visitate `n` volte, e . In questo modo sarà possibile generare `heatmap` di esplorazione del `maze` sulla base del numero di visite per cella, e `barplot` del numero totale di celle visitate. Inoltre verrà salvato anche il percorso minimo individuato `offline` sia della mappa interna costruita dall'agente, sia della mappa completa del `maze`, per permettere un confronto qualitativo tra i due percorsi.  

Ci aspettiamo che A* individui il percorso minimo, tuttavia visiti le medesime celle con un alta frequenza, risultando in una minore efficienza rispetto a strategie di depth-first search come flood-fill. Chiaramente i risultati dipenderanno dalla configurazione del `maze` e dalla capacità di scalare efficacemente da parte degli algoritmi. 

## 5. Suddivisione del lavoro

Da definirsi in corso d'opera in base alle competenze e preferenze dei membri del gruppo, con l'obiettivo di garantire una distribuzione equa delle attività e una collaborazione efficace. Potrebbe essere utile suddividere il lavoro in fasi, ad esempio:

- fase 1: studio del problema e familiarizzazione con il simulatore `mms` (tutti i membri);
- fase 2: implementazione degli algoritmi di esplorazione (suddivisi tra i membri);
- fase 3: costruzione e aggiornamento della mappa interna del labirinto (suddiviso tra i membri);
- fase 4: gestione dei casi di dead-end e scelta del prossimo obiettivo di esplorazione (suddiviso tra i membri);
- fase 5: valutazione su maze diversi e confronto tra strategie (tutti i membri);
- fase 6: analisi critica dei risultati e stesura della relazione finale (tutti i membri).
