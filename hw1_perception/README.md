# Homework 1 - Perception

## Aspetti principali

- Configurazione *Rviz* per meglio visualizzare alcuni dati nell'ambiente di lavoro e i risultati ottenuti
- *Listener* per [apriltags](http://wiki.ros.org/apriltags_ros) sul topic `/tag_detections` per ottenere contemporaneamente, per ciascun oggetto nel campo visivo della Kinect:
    - ID del tag
    - Size del tag
    - Pose rispetto a `camera_rgb_optical_frame`
- Trasformazione delle coordinate dal frame della camera al frame della base del robot
- Creazione di due *publisher* che comunicano, rispetto al `base_frame`
  (o al frame originale della camera, in caso di errori nella computazione della trasformata) del robot:
    1. le posizioni dei tag richiesti da riga di comando, se presenti
    2. le posizioni dei tag *NON* richiesti, se presenti
- Scrittura in un file `output.txt` (dentro la cartella del modulo) delle informazioni dei tag richiesti, come da consegna

## Modalità di funzionamento

In shell separate lanciare:

- La challenge_arena in Gazebo:  
    ```
    roslaunch challenge_arena challenge.launch sim:=true
    ```
- Il modulo apriltags:  
    ```
    roslaunch challenge_arena apriltag.launch
    ```
- Rviz per visualizzare i risultati *(opzionale)*:  
    ```
    rosrun rviz rviz -d `rospack find hw1_perception`/rviz/our_config.rviz
    ```
- Per utilizzare effettivamente quanto sviluppato:  
    ```
    roslaunch hw1_perception discover.launch 
        [ids:="[[frame_id],]"] [sim:=true/false] [forever:=true/false]
    ```
    
    - Il parametro `ids` contiene la lista dei tag (separati da `,`) che si vuole cercare sul tavolo:
        se trovati, vengono salvati all'interno del file di output e pubblicati dal publisher corrispondente.
        Altri tag trovati ma non richiesti vengono pubblicati dall'altro publisher dedicato.
        Se il parametro non viene fornito, saranno pubblicati e salvati tutti i tag trovati.
        Se si inseriscono valori non validi saranno trascurati e verrà visualizzato sulla shell un messaggio di errore.
    - Il parametro `sim` discrimina tra ambiente reale o simulato per poter utilizzare i parametri di offset e migliorare la precisione.
    - Il parametro `forever` viene utilizzato per non terminare l'esecuzione dopo un solo ciclo di ricerca dei frame
        (il file di testo non viene toccato se impostato a `true`).

Note aggiuntive
    
- La lista dei nomi dei tag si può trovare nella costante `tagnames` all'inizio di [discover.cpp](src/discover.cpp)
- Dopo aver fatto partire i vari moduli, spostare qualche oggetto dal tavolo di supporto alla superficie sotto la camera per vedere a terminale la pubblicazione e l'elaborazione dei messaggi
- I topic pubblicati possono essere visualizzati in Rviz o tramite shell:  
	```rostopic echo /tags_to_grab```
	
	```rostopic echo /tags_to_avoid```  
	
	Vengono pubblicate solo le pose e *non* i rispettivi id dei tag, poiché in quest'ultimo caso oltre a un messaggio *custom* occorre creare un plugin esterno per la visualizzazione in Rviz (`rostopic` funziona regolarmente).

- I parametri hanno tutti valori di default:  
    
    - ```sim:=true```  
    - ```forever:=false```  
    - ```ids:=""```  
    
- Se vengono passati *solamente* id NON validi verranno considerati tutti validi.
- Se vengono passati boolean NON validi (a `sim` o `forever`), verrà adottato il comportamento di default.
