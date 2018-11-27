# Robotica autonoma

Gruppo 01: Elia Bonetto, Matteo Dal Zovo, Filippo Rigotto

Questo repository contiene i file da aggiungere al workspace ROS per la challenge.

Ogni sottocartella è provvista di file README con le *istruzioni dettagliate* per l'uso dello specifico pacchetto.

## Setup

Si assume che ROS sia installato e correttamente configurato.
Si ipotizza che un workspace sia inizializzato in `~/ros_ws` per brevità nei comandi successivi.

```bash
cd ~/ros_ws/src/G01
git init
git remote add origin https://[USER]@bitbucket.org/iaslab-unipd/g01-bonetto-dal-zovo-rigotto.git
git pull origin master
```

Per utilizzare i pacchetti è necessario compilare il codice

```bash
cd ~/ros_ws
catkin_make
```
