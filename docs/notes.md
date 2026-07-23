# Implementation notes
A running module must be implemented in which it is possible to choose from a set of mazes and algorithms to run the simulation with mms maze solver simulator. 

**Materiale di riferimento**:
- art. 1 [Micromouse 3D simulator with dynamics capability: a Unity environment approach]([articles/s42452-021-04239-7.pdf](https://link.springer.com/article/10.1007/s42452-021-04239-7))
- art. 2 [Optimizing Tremaux Algorithm in Micromouse Using Potential Values]([articles/7_Sanjaya_Vol3_No2.pdf](https://www.researchgate.net/publication/361212084_International_Journal_of_Advanced_Engineering_Optimizing_Tremaux_Algorithm_in_Micromouse_Using_Potential_Values))
- video [Virtual Micromouse Maze Mapping and Solving Demonstration](https://www.youtube.com/watch?v=6y4nrnfZ1k0)
- video [Micromouse Maze Simulator - 2018 Japan Halfsize(32x32)](https://www.youtube.com/watch?v=-r8a8aPRYAQ)
- video [Micromouse Maze simulation](https://www.youtube.com/watch?v=0YId4SPJrWo)
- post [Micromouse from scratch](https://medium.com/@minikiraniamayadharmasiri/micromouse-from-scratch-algorithm-maze-traversal-shortest-path-floodfill-741242e8510)
- post [Floodfill Module](https://projects.ieeebruins.com/micromouse/floodfill-module)

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