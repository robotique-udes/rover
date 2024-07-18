# Network Configuration

Here you will find the ip addresses and login info of each antenna as well as all the used IP addresses for all components.

## IP address table

The rover is on the 192.168.144.0/24

Personnal computers must be on IP addresses between 192.168.144.100 and  192.168.144.254.

This is a table of all IP addresses used.

| IP address     | Composante               |
|----------------|--------------------------|
| 192.168.144.20 | Ordinateur de bord (MCU) |
| 192.168.144.25 | Caméra R1M 1             |
| 192.168.144.26 | Caméra R1M 2             |
| 192.168.144.50 | Rover M2.4               |
| 192.168.144.55 | Base station M2.4        |

The ROS_DOMAIN_ID must be 69 on all computers :

```bash
export ROS_DOMAIN_ID=69
```

## Antennas

The user and password for all the antennas is *rovus*.

### Rocket M2

* IP rover (Rover M2): 192.168.144.50
* IP base station (Base M2): 192.168.144.55
* SSID : rovusM2

Interface de configuration:

* Username: rovus
* Password: rovus

### Rocket M900 (old)

* IP rover (Rover M900) : 192.168.143.50 (Rick)
* IP base station (Base M900) : 192.168.143.55 (Morty)
* SSID : rovusM900

### Branchement de l'antenne

* Brancher l'antenne 2.4GHz dans l'ordinateur
* S'assurer que la configuration réseau de l'ordinateur est bien faite:
  * IP entre ]192.168.144.100 ; 192.168.144.255]
  * Netmask: 255.255.255.0
  * Pas d'adresse de base
* Ping l'antenne de la base station pour s'assurer qu'elle est bien connectée
* Ping l'antenne du rover pour s'assurer que la communication se fait bien

### Wireless Setup

Il est possible de se connecter au Rover sans sortir l'antenne de la base station, très pratique pour des tests à courtes portées. Il y a un router en mode lan repeter dans le rover qui émet un réseaux wifi qui permet d'accèder au réseaux local du rover (m2.4 + MCU)

Information de connections:

* Interface de configuration du router
  * password: rover

* Réseau wifi
  * SSID: rovus_low_range
  * password: roverrover
