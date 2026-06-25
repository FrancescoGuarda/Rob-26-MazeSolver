# 3 - (Pratico) Micromouse con simulatore di maze

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

**Materiale di riferimento**:
- art. 1 [Micromouse 3D simulator with dynamics capability: a Unity environment approach](articles/s42452-021-04239-7.pdf)
- art. 2 [Optimizing Tremaux Algorithm in Micromouse Using Potential Values](articles/7_Sanjaya_Vol3_No2.pdf)
- video [Virtual Micromouse Maze Mapping and Solving Demonstration](https://www.youtube.com/watch?v=6y4nrnfZ1k0)
- video [Micromouse Maze Simulator - 2018 Japan Halfsize(32x32)](https://www.youtube.com/watch?v=-r8a8aPRYAQ)
- video [Micromouse Maze simulation](https://www.youtube.com/watch?v=0YId4SPJrWo)

# Proposta progetto di Robotica 

## 1. Obiettivo del progetto

Nelle competizioni di micromouse, l'obbiettivo principale è quello di esplorare e mappare parzialmente o completamente il `maze` **online** (**full exploration**) per poi identificare il percorso minimo **offline** per raggiungere il goal dallo stato iniziale entro i limiti di quanto esplorato. Nell'ambito di questo progetto, ci poniamo nello scenario in cui l'obbiettivo dell'agente è quello di raggiungere il goal (di cui si assume conoscere le coordinate), riducendo al minimo la fase di esplorazione del `maze`, prioritizzando l'efficienza ed efficacia nell'individuazione di un percorso, anche sub-ottimo, per raggiungere il goal (evitando **full exploration**).

Nel dettaglio, l'obbiettivo del progetto è quello di implementare e testare differenti strategie di esplorazione del `maze`,ad esempio:  wall-following, flood fill e A*; e confrontarne le prestazioni in termini di efficienza ed efficacia nell'individuazione di un percorso per raggiungere il goal. Poichè l'agente si concentra nell'individuazione di un percorso, la ricerca **offline** del percorso minimo non rientra negli obiettivi dell'agente, ma viene comunque impiegata come indicatore nelle modalità descritte al punto [4](#4-risultati-attesi-e-metriche-di-valutazione).

L'obiettivo è quello di valutare le strategie sopra citate, nei termini di efficacia ed efficienza descritti al punto [4](#4-risultati-attesi-e-metriche-di-valutazione), in relazione a differenti configurazioni del `maze` a complessità crescente e casi critici/sintomatici (ad esempio labirinti con dead-end, loop, isole, ecc.) estratti dalle collezioni di labrinti messi a disposizioni per le competizioni ufficiali di micromouse (ad esempio [`maze-collection`](https://www.tcp4me.com/mmr/mazes/)), come descritto nella sezione [3](#3-scenari-di-valutazione).

Di seguito è riportata una plausibile suddivisione dei contenuti della relazione finale:

**Table of contents (relazione)**:
- Descrizione del problema di MicroMouse e dell'interfaccia del simulatore `mms`
- Descrizione degli algoritmi di esplorazione implementati (wall-following, flood fill, A*) ad alto livello
- Confronto delle mappe costruite dagli algoritmi di esplorazione su sottoinsieme di labirinti del test set
- Descrizione gestione dei casi limite (dead-end, loop, next-point) secondo le strategie implementate
- Descrizione dell'effettiva implementazione delle strategie di esplorazione 
- Presentazione dei casi di test, e analisi critica dei risultati ottenuti, in relazione alle metriche di valutazione definite.
- Conclusioni e possibili sviluppi futuri

## 2. Componenti software

- simulatore di `maze` : [`mms`](https://github.com/mackorone/mms)
- maze file collection : [`maze-collection`](https://www.tcp4me.com/mmr/mazes/)

## 3. Scenari di valutazione

Gli algoritmi saranno testati su un set di labirinti di complessità crescente, selezionati dalla collezione `maze-collection`. Verranno selezionati labirinti con caratteristiche diverse, come ad esempio labirinti con dead-end, loop, isole, ecc., al fine di valutare le prestazioni degli algoritmi in scenari diversi e mettere in evidenza eventuali punti di forza e criticità delle strategie implementate (es., wall-following fallisce in labirinti con isole).

## 4. Risultati attesi e metriche di valutazione

**efficienza di esplorazione**: numero totale di mosse necessarie per raggiungere il goal 

**efficacia di esplorazione**: numero totale di celle distinte visitate per raggiungere il goal (porzione di `maze` esplorata)

> Importante definire se intendiamo **efficienza** ed **efficacia** nell'esplorazione del `maze` intero o nel raggiungimento del goal.
( perchè se si intende l'esplorazione totale del `maze` metodi come A* non avrebbero senso, in quanto basati su euristiche che guidano l'esplorazione verso il goal, e non verso l'esplorazione totale del `maze`; altrimenti si ridurrebbe a BFS)

- `heatmap` di esplorazione del `maze` sulla base del numero di visite per cella
- `barplot` numero totale di celle visitate 

## 5. Suddivisione del lavoro

TODO: *riscrivere correttamente*

Da definirsi in corso d'opera in base alle competenze e preferenze dei membri del gruppo, con l'obiettivo di garantire una distribuzione equa delle attività e una collaborazione efficace. Potrebbe essere utile suddividere il lavoro in fasi, ad esempio:
- fase 1: studio del problema e familiarizzazione con il simulatore `mms` (tutti i membri);
- fase 2: implementazione degli algoritmi di esplorazione (suddivisi tra i membri);
- fase 3: costruzione e aggiornamento della mappa interna del labirinto (suddiviso tra i membri);
- fase 4: gestione dei casi di dead-end e scelta del prossimo obiettivo di esplorazione (suddiviso tra i membri);
- fase 5: valutazione su maze diversi e confronto tra strategie (tutti i membri);
- fase 6: analisi critica dei risultati e stesura della relazione finale (tutti i membri).
