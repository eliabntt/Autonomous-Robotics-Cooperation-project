# Homework 4 - Macchina a stati finiti

## Aspetti principali

## Modalità di funzionamento

In shell separate lanciare:

- La challenge_arena in Gazebo:  
    ```
    roslaunch challenge_arena challenge.launch sim:=true
    ```
- Rviz per visualizzare i risultati *(opzionale)*:  
    ```
    rosrun rviz rviz -d `rospack find g01_move`/rviz/marrtino_config.rviz
    ```
- Per utilizzare effettivamente quanto sviluppato:  
    ```
    roslaunch g01_fsm challenge.launch 
        [ids:="[[frame_id],]"] [sim:=true/false]
    ```
    
    - Il parametro `ids` contiene la lista dei tag (separati da `,`) che si vuole cercare sul tavolo:
        se trovati, vengono salvati all'interno del file di output e pubblicati dal publisher corrispondente.
        Altri tag trovati ma non richiesti vengono pubblicati dall'altro publisher dedicato.
        Se il parametro non viene fornito, saranno pubblicati e salvati tutti i tag trovati.
        Se si inseriscono valori non validi saranno trascurati e verrà visualizzato sulla shell un messaggio di errore.
    - Il parametro `sim` discrimina tra ambiente reale o simulato per poter utilizzare i parametri di offset e migliorare la precisione.

## Note aggiuntive
    
- La lista dei nomi dei tag si può trovare nella costante `tagnames` in `tags.h`, parte del pacchetto perception.
- I parametri hanno tutti valori di default:  
    
    - ```sim:=true```  
    - ```ids:=""```  
    
- Se vengono passati *solamente* id NON validi verranno considerati tutti validi.
- Se verrà passato un valore NON valido a `sim`, verrà adottato il comportamento di default.
