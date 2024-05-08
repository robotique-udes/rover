# Network Configuration

Here you will find the ip addresses and login info of each antenna as well as all the used IP addresses for all components.

## IP address table

The IP addresses for all components using 900MHz communication is 192.168.143.XXX. For the 2.4GHz, it is 192.168.143.XXX.

Personnal computers must be on IP addresses between 192.168.14X.100 and  192.168.14x.254.

This is a table of all IP addresses used.

| IP address     | Composante               |
|----------------|--------------------------|
| 192.168.143.50 | Rover M900               |
| 192.168.143.55 | Base staion M900         |
|                |                          |
| 192.168.144.20 | Ordinateur de bord (MCU) |
| 192.168.144.25 | Caméra R1M 1             |
| 192.168.144.26 | Caméra R1M 2             |
| 192.168.144.50 | Rover M2.4               |
| 192.168.144.55 | Base station M2.4        |

## Antennas

The user and password for all the antennas is *rovus*.

### Rocket M900

* IP rover (Rover M900) : 192.168.143.50 (Rick)
* IP base station (Base M900) : 192.168.143.55 (Morty)
* SSID : rovusM900

### Rocket M2

* IP rover (Rover M2): 192.168.144.50
* IP base station (Base M2): 192.168.144.55
* SSID : rovusM2

### Branchement des antennes

* Brancher l'antenne 2.4GHz dans l'ordinateur
* Brancher l'antenne 900MHz dans l'ordinateur
* S'assurer que la configuration réseau de l'ordinateur est bien faite
* Ping les deux antennes pour s'assurer qu'elles sont bien connectées
* Ping les deux antennes du rover pour s'assurer que la communication se fait bien

Il est important de brancher la 2.4GHz avant la 900MHz, car sinon la 2.4GHz ne sera pas reconnue par l'ordinateur.