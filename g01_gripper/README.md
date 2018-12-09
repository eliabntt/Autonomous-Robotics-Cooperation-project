# Homework 2 - MoveIt
N.B. Bisogna fare il pull di `g01_perception` e ricompilarlo per poter ottenere i messaggi che verranno utilizzati all'interno di questo homework.

## Aspetti principali
Verrà lanciata la parte di percezione che pubblicherà due topic separati uno per gli oggetti che devono essere trasportati, uno per gli oggetti che devono essere evitati.
Dopo l'inizializzazione il robot si porterà in una posizione zero con il gripper aperto a quel punto inizializzerà la scena inserendo gli oggetti di collisione e due muri perimetrali.

Gli ottagoni sono stati modellati per comodità come parallelepipedi. 

Una volta ottenuto la lista degli oggetti da spostare si procede partendo dagli ottagoni per poi procedere con i cubi e i prismi triangolari sulla zona finale dove verranno rilasciati.

La posizione zero è data in angoli per ciascun giunto mentre le traiettorie dei movimenti generali vengono calcolate su punti cartesiani intermedi così da riuscire a forzare per quanto possibile il movimento lungo una linea minimizzando le perdite di tempo per quanto possibile.

In simulazione è richiesta una particolare routine per poter visualizzare in Gazebo i movimenti degli oggetti.



## Modalità  di funzionamento


```
roslaunch challenge_arena challenge.launch sim:=true
```

```
roslaunch ur10_platform_challenge_moveit_config ur10_platform_challenge_moveit_planning_execution.launch sim:=true
```

```
roslaunch challenge_arena apriltag.launch sim:=true
```

```
roslaunch g01_perception discover.launch [ids:="[[frame_id],]"] forever:=true
```

```
catkin_make && roslaunch g01_gripper grip.launch
```


Non necessario:
```
roslaunch ur10_platform_challenge_moveit_config moveit_rviz.launch sim:=true
```
### Note aggiuntive

Una seconda configurazione del launch file permette di lanciare direttamente la fase di percezione dal programma di *pick and place*. Abbiamo preferito mantenere separato in quanto *g01_perception* verrà lanciato in una scheda esterna e non nel calcolatore principale.

Il programma è stato sviluppato nella sua interezza su una classe separata incapsulando per quanto possibile le varie procedure ripetitive all'interno di funzioni separate dove possibile.

Il planning viene ripetuto nel caso non si ottenga un risultato soddisfacente alla prima iterazione modificando il numero di punti intermedi. Se si ottiene un risultato abbastanza alto si esegue immediatamente, altrimenti si ripete il *planning* e si esegue solo se la percentuale di successo è comunque sopra una certa soglia. 
Nel caso di fallimento totale si abortisce il tentativo tornando alla posizione zero. Si riproverà in seguito una volta che sono stati spostati gli altri pezzi.
