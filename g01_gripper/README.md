# Homework 2 - UR10 + Gripper

Questo homework per funzionare necessita della versione più aggiornata del primo homework `g01_perception`.

## Aspetti principali

Verrà dapprima lanciata la parte di percezione che pubblicherà due topic separati: uno per gli oggetti che devono essere trasportati, uno per gli oggetti che devono essere evitati.
Sono stati creati messaggi _custom_ per la comunicazione del nome dei tag insieme alla pose, e quindi ora i topic `/tags_to_grab|avoid` pubblicano un array di PoseStamped.
L'header principale del messaggio contiene il frame di riferimento delle letture, `/world`, e l'header di ogni oggetto PoseStamped contiene il nome del tag nel `frame_id`.

Quando pronto, il robot si porterà in posizione "zero" con il gripper aperto;
il programma rimane poi in attesa di ricevere messaggi dai topic al più per 5 secondi, poi termina.

La ricezione è un elemento bloccante: senza oggetti sul tavolo non si raggiunge il secondo step del programma;
solo a quel punto viene inizializzata la scena inserendo gli oggetti di collisione.

I muri perimetrali in simulazione vengono inizializzati solo *dopo* che il robot si è portato in posizione "zero" in quanto
la posizione di partenza sarebbe in collisione con lo stesso. In caso di robot reale vengono invece inizializzati immediatamente
per evitare comportamenti scomposti del braccio stesso.

A ciascun oggetto viene associata una tripletta di valori rappresentanti le sue dimensioni e quindi possiamo distinguere tra cubi, parallelepipedi e prismi.
I parallelepipedi a base esagonale sono stati modellati per comodità con una base quadrata essendo loro larghi al più come il cubo.
I prismi a sezione triangolare sono stati modellati tramite la rispettiva mesh presente all'interno della *challenge arena*.

Una volta ottenuto la lista degli oggetti da spostare si procede partendo dai parallelepipedi, cioè gli oggetti più ingombranti,
per poi procedere con i cubi e infine con i prismi triangolari trasportandoli tutti per il momento sopra la zona finale dove verranno rilasciati.

Solo la posizione "zero" è data in angoli per ciascun giunto, le traiettorie dei movimenti nella routine *pick and place* vengono
calcolate su punti cartesiani intermedi così da cercare di forzare un movimento lineare, minimizzando quindi le perdite di tempo per quanto possibile.


## Modalità di funzionamento (in simulazione)

In terminali separati lanciare: la challenge_arena in Gazebo, il pianificatore per il manipolatore, il modulo apriltag, il modulo perception dell'homework precedente, questo modulo.

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
roslaunch g01_perception discover.launch [ids:="[[frame_id],]"] forever:=true sim:=true
```

```
roslaunch g01_gripper grip.launch sim:=true
```

Alternativamente ai due precedenti:

```
roslaunch g01_gripper grip_perc.launch sim:=true [ids:="[[frame_id],]"]
```

Per vedere la scena in Rviz (non strettamente necessario)
```
roslaunch ur10_platform_challenge_moveit_config moveit_rviz.launch sim:=true
```

### Note aggiuntive

Una secondo launch file permette di lanciare direttamente la fase di percezione (modulo perception) dal programma di *pick and place*.
Abbiamo però preferito usare i due in modo separato in quanto *g01_perception* viene poi lanciato in una scheda esterna e non nel calcolatore principale.

Il programma è stato sviluppato su una classe separata, incapsulando le varie procedure ripetitive all'interno di funzioni separate dove possibile.

Il planning viene ripetuto nel caso non si ottenga un risultato soddisfacente alla prima iterazione, modificando il numero di punti intermedi.
Se si ottiene un risultato abbastanza alto si esegue immediatamente, altrimenti si ripete il *planning* e si esegue solo se la percentuale di successo è comunque sopra una certa soglia.
Nel caso di fallimento totale si abortisce il tentativo tornando alla posizione zero.
Si riproverà la presa in seguito, per un massimo di cinque iterazioni, dopodichè i pezzi rimasti sul tavolo verranno ignorati.

Per la fase di presa dell'oggetto, in simulazione è richiesto l'agganciamento (e lo sganciamento) manuale dell'oggetto al gripper tramite link fittizio,
eseguito con una chiamata a un servizio esposto da Gazebo.

Il controllo della chiusura delle dita è attualmente in fase di sperimentazione in quanto il flag *gSTA* non esprime un comportamento assimilabile
a quanto sarebbe riscontrabile nel robot reale. Attualmente quindi in simulazione le dita vengono considerate come
correttamente chiuse, il caso reale verrà testato in seguito.

Per il modello della scatola il lavoro attualmente è in un branch separato in attesa di più dettagli su *marrtino* in modo da
poterlo esprimere parametricamente in base alla posizione raggiunta dal robot. Un'idea aggiuntiva sarebbe quella di utilizzare una posizione data in
giunti sopra la LZ e poi aggiustare la discesa finale.