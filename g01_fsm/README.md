# Homework 4 - FSM

## Aspetti principali

In questo homework è stata aggiunta la macchina a stati finiti per comandare lo svolgimento dell'intera challenge.
Funziona tramite lo scambio di messaggi tra questo e il modulo del manipolatore sul topic `g01_fsm_state`.

Quando marrtino raggiunge l'imboccatura del corridoio viene pubblicato il comando di risveglio della parte percettiva del modulo del manipolatore: le posizioni dei tag vengono lette e elaborate in modo da ottimizzare i tempi di esecuzione (la lettura nel reale impiega 4-5 secondi in media).
Quando marrtino raggiunge la zona di carico, viene dato il via libera allo spostamento degli oggetti al manipolatore.
La posa del marrtino viene utilizzata per creare una matrice di posizioni dove dovranno essere appoggiati gli oggetti:
in base allo stato di occupazione della scatola posta in cima a marrtino e ai pezzi eventualmente mancanti, il modulo del manipolatore comunicherà se è necessario un secondo giro per completare il trasferimento di tutti i pezzi.
In caso positivo, una volta raggiunta la zona di scarico marrtino rimarrà in attesa sul topic `g01_start_run` del via libera per poter cominciare un altro giro.
Questi comportamenti vengono ripetuti in loop fino al completo trasferimento di tutti gli oggetti necessari.

## Modalità di funzionamento (in simulazione)

Questo modulo contiene anche i due launch file necessari per far partire l'intera challenge: oltre a gazebo e rviz, lanciare prima il gruppo dei planner (apriltag, moveit, marrtino) e poi quello di questi moduli:

```
roslaunch g01_move challenge_planners.launch sim:=true
```

```
roslaunch g01_move challenge_packages.launch sim:=true ids:="[[frame_id],]"
```

Per far cominciare un secondo giro, quando servono ulteriori pezzi, usare il comando

```
rostopic pub --once g01_start_run std_msgs/Bool 'true'
```

Non ha effetto se il task è stato completato correttamente.

### Note aggiuntive

Sono state aggiunte routine apposite per il docking in zona di carico e di scarico e per effetturare una rotazione in caso di necessità di eseguire un secondo giro.
Questa rotazione verrà effettuata per orientare il marrtino verso la direzione di approccio precedentemente utilizzata(sicuramente libera).

Per evitare problemi di perdita di localizzazione è stata utilizzata una configurazione dinamica della precisione voluta dell'angolo di goal nel primo checkpoint e nell'ultimo, cioè quelli per cui la direzione d'approccio non è e non può essere deterministica.
