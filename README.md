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

### Test du Matériel : `Motor_Encoder_Test.ino`
Ce code de test permet de valider le bon fonctionnement du matériels.
* **Vérification du capteur mpu6050:** Ce sketch lit et affiche dans le Moniteur Série les valeurs brutes d'accélération des 3 axes et calcul l'angle sur l'axe X.
* **Contrôle de la Motorisation:** Il fait tourner les moteurs et affiche dans le Moniteur Série la position des moteurs.

### Premier Niveau de Contrôle : `SBRobot_Immobile_v1.0.0.ino`
Ce code implémente une machine d'état simple pour garder le robot en mode stationnaire.

* **Mode de Calibration:** Au démarrage, Si le robot est a l'envers, il rentre dans un état de calibration. Il va lire les données du capteur MPU pendant quelques secondes lorsqu'il est posé à plat sur une surface stable pour définir les offset du capteur pour avoir une référence à (0°). Il va aussi calibré le PWM minimum pour que les moteur commence a tourner. Toutes infomations du la calibration s'affiche dans le Moniteur Série et s'enregistrera dans la mémoire de l'ESP32.

* **Contrôle PID (Proportionnel, Intégral, Dérivé):** Une fois calibré, le robot passe dans un état de contrôle. L'algorithme PID compare en permanence l'inclinaison mesurée à la consigne (0°) et calcule la vitesse et le sens de rotation des roues à appliquer pour contrecarrer la chute, maintenant ainsi le robot en position verticale et immobile.
* **Affichage sur l'Écran TFT:** Cette version utilise l'écran TFT intégré de 2,9 pouces pour afficher en temps réel des informations clés sur l'état du robot, telles que l'angle actuel, l'état de la calibration, et éventuellement la sortie du PID ou des messages d'erreur.

---
