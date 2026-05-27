# 🤖 Elektor Mini-Wheelie : Projet 2601 Robot self-balancing

> **Documentation de suivi de projet et d'implémentation**
> *Basé sur le kit Makerfabs / Elektor*

**Ressources Officielles :**
* [Boutique : Elektor Mini-Wheelie Self-Balancing Robot](https://www.elektor.com/products/elektor-mini-wheelie-self-balancing-robot?utm_source=elektormagazine.com&utm_medium=ProductLink&utm_campaign=english-feed%20&utm_content=Elektor%20Mini-Wheelie%20Self-Balancing%20Robot)
* [Code et documentation fournis par le fabricant](https://www.elektormagazine.com/labs/self-balancing-robot-with-maker-fabs)

---

## Objectif du Projet
Basé sur l'ESP32-S3, ce robot éducatif permet d'explorer la régulation PID pour l'auto-équilibrage. Il intègre un module inertiel MPU-6050 pour la détection d'inclinaison, un capteur ultrasonique HC-SR04 pour l'évitement d'obstacles et un écran TFT de 2,9 pouces pour l'affichage des informations.

---

## Environnement et Versionning

* **Microcontrôleur** : ESP32-S3 
* **IDE** : [Arduino IDE version 2.3.7](https://www.arduino.cc/en/software)
* **Application de contrôle** : [Dabble](https://thestempedia.com/product/dabble/) (disponible sur iOS & Android) via Bluetooth.
* **Langage** : C++ (Framework Arduino)

---

## Mise en place du projet et Déploiement

### 1. Prérequis et Installation
1. Téléchargez et installez l'éditeur [Arduino IDE v2.3.7](https://www.arduino.cc/en/software).
2. Installer l'ESP32 et les librairies dans Arduino IDE (voir page 9/13 du [manuel d'assemblage et installation](https://cdn.shopify.com/s/files/1/0626/6280/3788/files/Construction_Manual_Elektor_Self-Balancing_Robot_V1.4.pdf?v=1745835509)).
3. Téléchargez l'application **Dabble** sur votre smartphone pour le contrôle via Bluetooth.

### 2. Configuration exacte de la carte (Important)
Pour compiler et téléverser le code vers l'ESP32-S3, allez dans le menu **Tools** d'Arduino IDE et configurez **exactement** les paramètres suivants :

* **Board** : `ESP32S3 Dev Module`
* **Port** : `COM13` *(Vérifiez le port attribué à votre robot sur votre PC)*
* **USB CDC On Boot** : `Enabled`
* **CPU Frequency** : `240MHz (WiFi)`
* **Core Debug Level** : `None`
* **USB DFU On Boot** : `Disabled`
* **Erase All Flash Before Sketch Upload** : `Disabled`
* **Events Run On** : `Core 1`
* **Flash Mode** : `QIO 80MHz`
* **Flash Size** : `4MB (32Mb)`
* **JTAG Adapter** : `Disabled`
* **Arduino Runs On** : `Core 1`
* **USB Firmware MSC On Boot** : `Disabled`
* **Partition Scheme** : `Default 4MB with spiffs (1.2MB APP/1.5MB SPIFFS)`
* **PSRAM** : `Disabled`
* **Upload Mode** : `UART0 / Hardware CDC`
* **Upload Speed** : `921600`
* **USB Mode** : `Hardware CDC and JTAG`

### 3. Déploiement et Flashage (Étape cruciale)
* Ouvrez le fichier `.ino` dans Arduino IDE.
* **Mise en mode Flash (Bootloader) :** Maintenez le bouton **BOOT** de la carte enfoncé, **branchez le câble USB** à votre PC, puis relâchez le bouton BOOT. *(Si cette manipulation n'est pas effectuée, l'IDE ne parviendra pas à flasher la carte et affichera une erreur de connexion).*
* Cliquez sur **Téléverser** dans l'IDE.

---

## Implémentation du Code et Développement Iteratif

Le développement du robot a été réalisé de manière progressive, en commençant par des tests unitaires matériels pour aboutir à un premier algorithme de contrôle d'auto-équilibrage.

### Premier Niveau de Contrôle : `SBRobot_Immobile_v1.0.0.ino`
Ce code implémente une machine d'état simple pour garder le robot en mode stationnaire.

* **Mode de Calibration:** Au démarrage, si le robot est à l'envers, il rentre dans un état de calibration. Il va lire les données du capteur MPU pendant quelques secondes lorsqu'il est posé à plat sur une surface stable pour définir les offsets du capteur afin d'avoir une référence à (0°). Il va aussi calibrer le PWM minimum pour que les moteurs commencent à tourner. Toutes les informations de la calibration s'affichent dans le Moniteur Série et s'enregistrent dans la mémoire flash de l'ESP32.
* **Contrôle PID (Proportionnel, Intégral, Dérivé):** Une fois calibré, le robot passe dans un état de contrôle. L'algorithme PID compare en permanence l'inclinaison mesurée à la consigne (0°) et calcule la vitesse et le sens de rotation des roues à appliquer pour contrecarrer la chute, maintenant ainsi le robot en position verticale et immobile.
* **Affichage sur l'Écran TFT:** Cette version utilise l'écran TFT intégré de 2,9 pouces pour afficher en temps réel des informations clés sur l'état du robot, telles que l'angle actuel, l'état de la calibration, et la sortie du PID.

---

## Logique Mathématique et Calculs Principaux

Le fonctionnement du robot repose sur trois concepts mathématiques majeurs, implémentés en boucle continue (à 100 Hz, soit toutes les 10 ms) pour assurer la stabilité :

### 1. Le calcul de l'inclinaison (Fusion de données)
Le module MPU6050 capte les mouvements, mais ses données brutes doivent être traitées pour obtenir un angle d'inclinaison fiable (`anglePitch`).

* **L'Accéléromètre** calcule un angle brut basé sur la gravité terrestre grâce à la trigonométrie (Arc Tangente) :
  $$accPitch =  rctan2(accX, accZ) 	imes rac{180}{\pi}$$
* **Le Gyroscope** mesure la vitesse de rotation en degrés par seconde (convertie selon la sensibilité matérielle du MPU, ici 131.0 LSB/°/s) :
  $$gyroRate = rac{-(gyroY - offGyroY)}{131.0}$$
* **Le Filtre Complémentaire** combine ces deux mesures. Il fait confiance au gyroscope sur le court terme (très réactif) et à l'accéléromètre sur le long terme (pour corriger la dérive gyroscopique) :
  $$anglePitch = 0.98 	imes (anglePitch + gyroRate 	imes dt) + 0.10 	imes accPitch$$

### 2. Le Régulateur PID (Auto-équilibrage)
Une fois l'angle connu, le robot doit déterminer avec quelle force faire tourner ses roues. C'est le rôle du PID. L'erreur $e(t)$ représente l'écart entre le point d'équilibre parfait (`setpoint = 0`) et l'angle actuel.

* **L'Action Proportionnelle ($K_p = 15$)** : Fournit une force de réaction immédiate. Plus le robot penche, plus il pousse fort.
  $$P = K_p 	imes e(t)$$
* **L'Action Intégrale ($K_i = 0.3$)** : Additionne les petites erreurs au fil du temps pour corriger un léger déséquilibre persistant (bridée entre -100 et 100 pour la sécurité matérielle).
  $$I = K_i 	imes \int e(t) dt$$
* **L'Action Dérivée ($K_d = 0.5$)** : Agit comme un amortisseur. Elle regarde à quelle vitesse le robot tombe pour anticiper et éviter qu'il n'oscille indéfiniment.
  $$D = K_d 	imes rac{e(t) - e(t-1)}{dt}$$
* **La Sortie ($pid\_output$)** : Est la somme de ces trois forces pour dicter l'ordre final aux roues.
  $$pid\_output = P + I + D$$

### 3. La commande matérielle des moteurs
Le résultat théorique du PID est ensuite converti en un signal de puissance de 0 à 255 (PWM) utilisable par les ponts en H des moteurs physiques.

* **Zone Morte (`commonMinPWM`)** : Si l'on envoie un courant trop faible à un moteur, il ne tourne pas à cause des frottements mécaniques. Le code ajoute donc une puissance de base pour compenser cela dès que le PID ordonne un mouvement.
* **Ratio de Synchronisation (`ratioLeft` / `ratioRight`)** : Aucun moteur physique n'est strictement identique. Les ratios compensent la différence de vitesse native entre la roue gauche et la roue droite pour garantir que le robot roule droit.
  $$Vitesse_{Finale} = (|pid\_output| 	imes Ratio) + MinPWM$$
