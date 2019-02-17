# Homework 3 - Marrtino

Questo homework per funzionare necessita delle versioni più aggiornate dei package dell'arena e di marrtino.

## Aspetti principali

Partendo dalla posizione iniziale, il robot usa il planner, dotato di configurazioni personalizzate, per raggiungere tre posizioni intermedie in sequenza, avvicinandosi sempre più all'entrata del corrodoio;
le posizioni sono al termine dell'area aperta e non troppo vicino ai muri, per lasciare al planner libertà d'azione, e compongono una traiettoria curvilinea, fino ad inserirsi all'inizio del corridoio.
Il corridoio è affrontato in modo manuale seguendo il muro sinistro fino al raggiungimento della zona di carico, dove il robot attende di essere caricato.
In seguito, marrtino viene ruotato di 180° verso destra; la rotazione non è esattamente sul posto, per allontanarsi dal muro sinistro, evitando contatti, e per evitare problemi di localizzazione.
Il ritorno avviene inizialmente utilizzando il planner sfruttando la buona localizzazione precedentemente ottenuta per avvicinarsi quanto più possibile all'ingresso del corridoio.
Successivamente, cercando di seguire sempre il muro sinistro, si prosegue fino all'uscita.
Marrtino viene fatto poi deviare leggermente a destra e ritorna quindi sui suoi passi, raggiungendo tramite planner le prime due posizioni intermedie in ordine inverso, e poi viene indirizzato verso la zona di scarico.

## Modalità di funzionamento (in simulazione)

In terminali separati lanciare l'arena in Gazebo e rviz.

```
roslaunch challenge_arena challenge.launch sim:=true
```

```
rosrun rviz rviz -d `rospack find g01_move`/rviz/marrtino_config.rviz
```

Questo modulo può essere lanciato tenendo la parte di navigazione separata (consigliato), con due terminali:

```
roslaunch g01_move robot_navigation.launch
roslaunch g01_move move.launch sim:=true
```

oppure con un singolo comando (l'output della navigazione va su log file):

```
roslaunch g01_move move_nav.launch sim:=true
```

**IMPORTANTE**: con l'aggiunta della macchina a stati finiti, questo modulo dipende da quello del manipolatore e quindi non è più possibile utilizzarlo separatamente (ossia, i comandi alla sezione precedente *non* portano a fare un giro completo).

### Note aggiuntive

- I file delle configurazioni personalizzate per planner e costmap sono contenuti in questo modulo e vengono utilizzati automaticamente dal file launch al posto dei predefiniti dell'arena.
