[Retour à la page d'accueil](/README.md)

# Configuration du drone

## Installation de adb / ssh

La plupart des outils de ModalAI utilisent adb pour se connecter au drone. Ainsi, il est important d'installer adb sur son ordinateur portable. Pour ce faire, il suffit de suivre les instructions sur le site web de ModalAI https://docs.modalai.com/setting-up-adb/

Une fois connecté au drone via adb, il est possible de le configurer pour se connecter à un réseau internet existant ou pour qu'il agisse en tant que routeur et fournisse son propre réseau. S'il est nécessaire de faire des mises à jours de systèmes sur le drone via apt-get, il faut que le drone soit connecté à un réseau existant pour avoir accès à l'internet. Pour configurer le wifi du drone, il suffit de suivre les étapes décrites ici : https://docs.modalai.com/voxl-2-wifi-setup/#configure-and-connect-in-station-mode. À noter qu'il existe deux modes, "Station", qui permet au drone de se connecter à un réseau, et "SoftAP", qui permet au drone d'agir comme routeur. 

Le drone permet aussi de se connecter en ssh. Le nom d'utilisateur est root et le mot de passe par défaut est oelinux123. 

## Installation nouvelle version / reset à zéro

Si jamais le drone est dans un état bricked ou qu'une mise à jour entière du drone est à effectuer, il est possible de réinitialiser le drone avec une version spécifique à l'aide de l'outil de ModalAI suivant : https://docs.modalai.com/flash-system-image/

Il est bon de prendre note que cette mise à jour préserve toutes les informations dans /data, donc les fichiers sku.txt, les fichiers de calibrations, les photos et vidéos prises, etc. mais réinitialise tous les autres fichiers à leur configuration par défaut d'usine. 

Il est aussi intéressant de noter que, suites à nos tests, le service de qvio ne fonctionne plus avec les caméras après la version voxl-suite 1.5.1. Ainsi, pour notre cas d'utilisation, à la date du Février 2026, la version 1.5.1 est la version où le qvio fonctionne encore la plus récente. 

La liste de tous les packages installés et leur version est disponible dans le dossier [documentation/configurations/version.txt](./configurations/version.txt).


## Configuration des paramètres.

### Sku
Avant de modifier la configuration du drone, il est important de vérifier le sku déjà existant dans le drone. Pour ce faire, on peut utiliser la commande ```voxl-inspect-sku```. Le SKU utilisé durant le projet est MRB-D0012-4-V2-C29-T9-M24-X0, (Le format est {family code}-{board code}-V{hw revision}-C{camera config}-T{transmitter code}-M{modem code}-X{extras bitmask}). Le SKU permet d'indiquer à voxl, le système d'exploitation du drone, quels sont les différentes composantes installées sur le drone.  

S'il faut modifier le SKU, on peut utiliser la commande : ```voxl-configure-sku --wizard```.  


### Extrinsics 
Les extrinsics permettent d'indiquer les emplacements des différentes composantes sur le drone, afin de permettre au drone de calculer sa position relative aux différentes caméras et autres capteurs. 

Pour configurer les extrinsics, il suffit d'utiliser la commande ```voxl-configure-extrinsics```. Les extrinsics présentement utilisés sont d0012_starling_2_max_c29_with_tof. 

### Autres configurations
En cas de besoin, il est possible d'obtenir la majorités des fichiers de configurations utilisés sur le drone durant le projet dans le dossier [documentation/configurations/modalai](./configurations/modalai/)

[Retour à la page d'accueil](/README.md)