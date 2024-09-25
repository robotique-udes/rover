# Bras 5DDl, 6 axes, 2024

## Overview

- [Jlin](#jlin): joint linéaire en x à la base du bras, permet un déplacement de 450 mm parallèle à l'avant du rover.
- [J0](#j0): Joint de rotation en z, permet une rotation de 360° de l'ensemble du bras (premier joint d'un bras normal)
- [J1](#j1): Joint de rotation équivalent à l'épaule d'un bras humain
- [J2](#j2): Joint de rotation équivalent au coude d'un bras humain
- [J3, J4](#j3-j4): Poignet différentiel qui permet la rotation sur l'axe du poignet et sur le plan de la pince
- [J5](#j5): Pince qui s'ouvre et se ferme

## Choix mécanique des joints

### Jlin

### J0

### J1

Choix de gearbox et moteur: <br>
Selon le fichier Excel, avec un payload de 10kg et un bras de 5kg, vitesse de 3RPM. Torque désiré = 350Nm (fs = 1.27, pas bcp) à 3RPM, avec une gearbox de 40:1, le point d'opération du moteur est à 120RPM et 8.75Nm. Le moteur d'essuie-glace 24V qui traînait était déjà bien sizé pour le joint. Gearbox 40:1 industrielle assez légère d'[Amazon](https://www.amazon.ca/-/fr/CNCTOPBAOS-NMRV030-R%C3%A9ducteur-vitesse-sans/dp/B09JNX9NH1?pd_rd_w=ieJzW&content-id=amzn1.sym.d66add78-05b0-4a3d-9404-5c9bc639cee0&pf_rd_p=d66add78-05b0-4a3d-9404-5c9bc639cee0&pf_rd_r=RMAV0PH891WQFF3QXR3Z&pd_rd_wg=xYh5F&pd_rd_r=574abcfb-23b4-450d-b03c-767a56e2bec0&pd_rd_i=B09JNX9NH1&ref_=pd_bap_d_grid_rp_0_1_ec_pd_hp_d_atf_rp_2_i&th=1 "Geabox NMRV030").

Bon coups:<br>
Moteur et gearbox bien sizée avec le moteur et le joint, mouvement très smooth.

À améliorer: <br>
Joint plutôt lourd (environ 1.15kg gearbox et 1kg moteur plus reste = 3kg) et limites physiques inconnues du moteur et du gearbox.

### J2

### J3, J4

### J5
