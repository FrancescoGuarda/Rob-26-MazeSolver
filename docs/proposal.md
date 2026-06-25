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

Testare e valutare l'efficienza ed efficacia di differenti algoritmi di esplorazioni di un `maze`, su un dataset di labirinti predefiniti estratti dalla collezione `maze-collection` impiegata come benchmark per le competizioni di Micromouse. Analizzare i risultat 

Nelle competizioni di micromouse, l'obbiettivo principale è quello di mappare completamente il `maze` **online** (**full exploration**) per poi identificare il percorso minimo **offline** per raggiungere il goal. 
Diversamente dalle competizioni standard, l'obbiettivo di questo progetto è quello di porsi in scenari 

Tuttavia, nello scenario in cui l'obbiettivo di un agente è quello di raggiungere i
L'obiettivo è quello di identificare le strategie che minimizzino la fase dell'esplorazione, piuttosto che l'identificazione del percorso minimo, ponendoci nello scenario in cui il goal dell'agente è raggiungere la zona obiettivo nella maniera più efficiente ed efficace possibile (e.g., valutando il numero di mosse, celle visitate), evitando l'esplorazione completa del `maze`, che rappresenta invece il problema standard nelle competizioni Micromouse. Una volta esplorato il `maze` infatti, l'identificazione del percorso minimo è un problema triviale, e esplorata la totalità del `maze` restituirebbe sempre il medesimo percorso minimo; In un contesto online invece, l'esplorazione completa del `maze` potrebbe non essere desiderabile, 

**Table of contents (relazione)**:
- Descrizione del problema di MicroMouse e dell'interfaccia del simulatore `mms`
- Descrizione degli algoritmi di esplorazione implementati (wall-following, flood fill, A* incrementale) ad alto livello
- Confronto delle mappe costruite dagli algoritmi di esplorazione su sottoinsieme di labirinti del test set
- Descrizione gestione dei casi limite (dead-end, loop, next-point)
- Descrizione dell'effettiva implementazione delle strategie di esplorazione 
- Presentazione dei casi di test, e analisi critica dei risultati ottenuti, in relazione alle metriche di valutazione definite.


## 2. Componenti software

- simulatore di `maze` : [`mms`](https://github.com/mackorone/mms)
- maze file collection : [`maze-collection`](https://www.tcp4me.com/mmr/mazes/)

## 3. Scenari di valutazione

I tre algoritmi citati al punto [1](#1-obiettivo-del-progetto) (wall-following, flood fill, A* incrementale) saranno implementati e testati su un set di labirinti di complessità crescente, selezionati dalla collezione `maze-collection`. La valutazione sarà basata sulle metriche definite al punto [4](#4-risultati-attesi-e-metriche-di-valutazione), con particolare attenzione alla capacità di minimizzare il numero di mosse necessarie per raggiungere il goal, evitando l'esplorazione completa del `maze` quando possibile.

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
