# Smart Home – Maison connectée sur STM32

Firmware d'une maquette de maison connectée sur carte **STM32 Nucleo-F411RE** : alarme intrusion, détection de gaz, mesure de température, et fenêtre motorisée qui se ferme toute seule quand il pleut.

Projet de cours de 4e année, ESIEE Paris (filière Systèmes Embarqués), réalisé en binôme avec Elyse Legal. Nous avons travaillé ensemble sur l'ensemble du projet, sans le découper en parties.
<!-- TODO : préciser le nom du cours -->

<!-- TODO : ajouter la photo de la maquette puis retirer ces commentaires -->
<!-- ![Maquette de la maison connectée](docs/images/maquette.jpg) -->

---

## Fonctionnalités

| Fonction | Capteur / actionneur | Comportement |
|---|---|---|
| Mode HOME / AWAY | Bouton B1 de la carte | Bascule entre maison occupée et maison vide |
| Alarme intrusion | Détecteur de mouvement PIR | Un mouvement déclenche l'alarme **uniquement en mode AWAY** |
| Alarme gaz | Capteur MQ-2 (ADC) | Toujours active, quel que soit le mode ; détecte aussi un capteur débranché |
| Température | Capteur TMP102 (I2C) | Mesure toutes les 2 s, en mode *one-shot* (une mesure à la demande) |
| Fenêtre motorisée | Servomoteur (PWM 50 Hz) + bouton | S'ouvre à la demande, **sauf s'il pleut ou en mode AWAY** |
| Détection de pluie | Capteur de pluie (ADC) | Force la fermeture de la fenêtre |
| Signalisation | LED clignotante + buzzer (PWM 2 kHz) | Actifs dès qu'une alarme est en cours |
| Supervision | UART 115200 bauds via le ST-LINK | Une ligne d'état complète toutes les 2 s |

Exemple de sortie sur le terminal série :
```
[ETAT] mode=AWAY | temp=21.50 C | gaz=1350 | alarme_gaz=OFF | intrusion=ON | pluie=150 (NON) | fenetre=CLOSED
```

---

## Points techniques

- **Interruptions courtes** : les interruptions (boutons, PIR, timer) se contentent de lever un drapeau ; tout le traitement se fait dans la boucle principale.
- **Base de temps matérielle** : le timer TIM2 déclenche une acquisition toutes les 2 s.
- **Anti-rebond logiciel** des boutons (200 ms), calculé avec l'horloge système plutôt qu'avec une attente bloquante.
- **Hystérésis et confirmation** sur le gaz et la pluie : deux seuils (déclenchement / fin d'alerte) et deux mesures consécutives exigées avant de changer d'état, pour éviter les alarmes qui clignotent autour du seuil.
- **Seuils calibrés à partir de nos propres mesures** (air ambiant, vapeurs de gel hydroalcoolique, capteur sec ou mouillé) ; positions du servo caractérisées sur la maquette.
- **Driver I2C du TMP102 écrit à la main** (`tmp102.c`) : trames commentées d'après la datasheet, gestion des erreurs et des délais d'attente.
- **Priorités de sécurité** : la position de la fenêtre est recalculée à chaque tour (demande de l'utilisateur ET pas de pluie ET mode HOME) ; le servo ne bouge que si le résultat change.
- Horloge à 100 MHz (quartz 8 MHz + PLL), configurée avec STM32CubeMX.

---

## Brochage

| Broche | Fonction |
|---|---|
| PA5 | LED LD2 (témoin de la base de temps) |
| PC13 | Bouton B1 – mode HOME/AWAY (EXTI) |
| PC10 | Détecteur PIR (EXTI) |
| PA9 | Bouton fenêtre (EXTI) |
| PC12 | LED d'alarme |
| PC7 | Buzzer – TIM3_CH2 (PWM 2 kHz) |
| PB6 | Servomoteur – TIM4_CH1 (PWM 50 Hz) |
| PA0 | MQ-2 – ADC1 canal 0 |
| PA1 | Capteur de pluie – ADC1 canal 1 |
| PB8 / PB9 | TMP102 – I2C1, adresse 0x48 |
| PA2 / PA3 | USART2 vers le ST-LINK |

---

## Organisation du dépôt

```
smart-home-stm32/
├── Core/
│   ├── Src/main.c      Logique de l'application (notre code)
│   ├── Src/tmp102.c    Driver du capteur de température (notre code)
│   └── ...             Fichiers d'initialisation générés par STM32CubeMX
├── Drivers/            Bibliothèques HAL et CMSIS de STMicroelectronics
└── ProjetDomotique_CS_EL.ioc   Configuration STM32CubeMX
```

Dans les fichiers générés par CubeMX, notre code se trouve entre les balises `USER CODE BEGIN` / `USER CODE END`.

## Compiler

Ouvrir le dossier dans **STM32CubeIDE** (*File → Import → Existing Projects into Workspace*), compiler, puis flasher la Nucleo-F411RE. Les messages s'affichent sur le port série du ST-LINK à 115200 bauds.
