# Robotica autonoma

## Homework 1

##### Aspetti principali:
- Creata configurazione *rviz* per meglio visualizzare l'ambiente di lavoro e i risultati ottenuti
- Creato *listener* per [*apriltags*](http://wiki.ros.org/apriltags_ros) sul topic **/tag_detections** per ottenere contemporaneamente, per ciascun oggetto nel campo visivo della Kinect:
      - ID del tag
      - Size del tag
      - Pose rispetto *camera_rgb_optical_frame*
- Trasformato le coordinate dal frame della camera al frame della base del robot
- Creato due *publisher* che comunicano, rispetto al *base_frame* del robot:
  1. le posizioni dei tag richiesti da riga di comando, se presenti
  2. le posizioni dei tag **NON** richiesti, se presenti
- Scritto in un file di output, come da consegna, le informazioni dei tag richiesti

##### Modalità di funzionamento:
Il programma per il momento non è provvisto di launch file. 
Assumendo che ```source [YOUR_CATKIN_WORKSPACE]/devel/setup.bash``` sia correttamente eseguito o automaticamente grazie al file `.bashrc` o manualmente per **ciascun terminale che necessita di lanciare ROS???**.

La cartella principale del programma deve essere localizzata all'interno di  ```[YOUR_CATKIN_WORKSPACE]/src ```. Ciò è possibile farlo semplicemente con:
```git
cd [YOUR_CATKIN_WORKSPACE]/src
git init
git remote add origin https://eliabntt@bitbucket.org/iaslab-unipd/g01-bonetto-dal-zovo-rigotto.git
git pull origin master
```

Fatto questo è possibile procedere con l'effettivo utilizzo del codice da noi sviluppato.

```bash
cd [YOUR_CATKIN_WORKSPACE]
catkin_make
```

In shell separate lanciare:

- Per la challenge_arena in Gazebo:  
    ```roslaunch challenge_arena challenge.launch sim:=true```
- Per apriltags:  
  ``` roslaunch challenge_arena apriltag.launch ```
- Per rviz(*opzionale*):  
   ```rosrun rviz rviz -d `rospack find hw1_perception`/rviz/our_config.rviz```
- Per utilizzare effettivamente quanto sviluppato:  
   ```roslaunch hw1_perception discover.launch ids:="[[frame_id],]" [sim:=true/false] [forever:=true/false]```  
    - Il parametro ```forever``` viene utilizzato per non terminare l'esecuzione dopo un solo ciclo di ricerca dei frame
    - Il parametro ```ids``` rappresenta la lista dei ```tagnames``` dei frame che vogliamo all'interno del file di output o all'interno del publisher corrispondente. Se non sono validi verrà visualizzato all'interno della console un messaggio di errore.
    - Il parametro ```sim``` indica il fatto se siamo in ambiente simulato o reale per poter utilizzare alcuni parametri in modo più efficiente
    
I topic pubblicati per il momento possono essere facilmente visualizzati in *rviz*.