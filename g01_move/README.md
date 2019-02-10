# Homework 3 - Marrtino, 4 - FSM

Questo homework per funzionare necessita delle versioni più aggiornate dei package dell'arena e di marrtino.

## Aspetti principali

Partendo dalla posizione iniziale, il robot usa il planner, dotato di configurazioni personalizzate, per raggiungere tre posizioni intermedie in sequenza, avvicinandosi sempre più all'entrata del corrodoio;
le posizioni sono al termine dell'area aperta e non troppo vicino ai muri, per lasciare al planner libertà d'azione, e compongono una traiettoria curvilinea.
Il corridoio è affrontato in modo manuale seguendo il muro sinistro fino al raggiungimento della zona di carico, dove il robot in via temporanea sosta per due secondi.
In seguito, marrtino viene ruotato di 180° verso destra; la rotazione non è esattamente sul posto, per allontanarsi dal muro sinistro e per evitare problemi di localizzazione.
Il ritorno avviene inizialmente utilizzando il planner sfruttando la buona localizzazione precedentemente ottenuta per avvicinarsi quanto più possibile all'ingresso del corridoio.
Successivamente, cercando di seguire sempre il muro sinistro, si prosegue fino all'uscita.
Marrtino viene fatto poi deviare leggermente a destra e ritorna quindi sui suoi passi, raggiungendo tramite planner le prime due posizioni intermedie in ordine inverso, e poi viene indirizzato verso la zona di scarico.

In questo homework è stata aggiunta la macchina a stati finiti per comandare lo svolgimento dell'intera challenge.
Funziona tramite lo scambio di messaggi tra questo e il modulo del manipolatore sul topic `g01_fsm_state`.

Quando marrtino raggiunge l'imboccatura del corridoio viene pubblicato il comando di risveglio della parte percettiva del modulo del manipolatore: le posizioni dei tag vengono lette e elaborate.
Quando marrtino raggiunge la zona di carico, viene dato il via libera allo spostamento degli oggetti al manipolatore:
in base allo stato di occupazione della scatola posta in cima a marrtino, il modulo del manipolatore comunicherà se è necessario un secondo giro per completare il trasferimento di tutti i pezzi.
In caso positivo, una volta raggiunta la zona di scarico marrtino rimarrà in attesa sul topic `g01_start_run` del via libera per poter cominciare un altro giro.
Questi comportamenti vengono ripetuti in loop fino al completo trasferimento degli oggetti necessari.

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

Per far cominciare un secondo giro, quando servono ulteriori pezzi, usare il comando

```
rostopic pub --once g01_start_run std_msgs/Bool 'true'
```

Non ha effetto se il task è stato completato correttamente.

### Note aggiuntive

- I file delle configurazioni personalizzate per planner e costmap sono contenuti in questo modulo e vengono utilizzati automaticamente dal file launch al posto dei predefiniti dell'arena.
L'unica eccezione è il launch file dell'amcl in quanto non sapevamo se la modifica fosse consentita.

- Questo modulo contiene anche i due launch file necessari per far partire l'intera challenge: oltre a gazebo e rviz, lanciare prima il gruppo dei planner (apriltag, moveit, marrtino) e poi quello di questi moduli:

```
roslaunch g01_move challenge_planners.launch sim:=true
```

```
roslaunch g01_move challenge_packages.launch sim:=true ids:="[[frame_id],]"
```