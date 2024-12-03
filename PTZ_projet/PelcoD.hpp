#ifndef ASSERVISSEMENT_PELCOD_H
#define ASSERVISSEMENT_PELCOD_H

#include <iostream>
#include <string>
#include <cstring>
#include <cmath>
#include <cstdint>

#ifdef __linux__ //Includes spécifiques à la version Linux
	#include <fcntl.h>
	#include <errno.h>
	#include <termios.h>
	#include <unistd.h>
#elif _WIN32 //Includes spécificques à la version Windows
	#include <windows.h>
#endif

//Sur Commande 1 (data[2])
#define bitFocusProche 0b00000001 // Code sur le 3e octet de commande pour rappocher le focus 
//Sur Commande 2 (data[3])
#define bitFocusLoin 0b10000000 // Code sur le 4e octet de commande pour éloigner le focus 
#define bitDezoom 0b01000000 // Code sur le 4e octet de commande pour dézoomer 
#define bitZoom 0b00100000 // Code sur le 4e octet de commande pour zoomer 
#define bitTiltBas 0b00010000 // Code sur le 4e octet de commande pour activer le tilt vers le bas 
#define bitTiltHaut 0b00001000 // Code sur le 4e octet de commande pour activer le tilt vers le haut 
#define bitPanGauche 0b00000100 // Code sur le 4e octet de commande pour activer le pan vers la gauche 
#define bitPanDroite 0b00000010 // Code sur le 4e octet de commande pour activer le pan vers la droite 



/**
    Protocole de communication Pelco_D via connexion USB

    Exemple :
            PelcoD pelco(ADRESSE_CAM, NOM_PORT);
            pelco.pan_tilt_zoom(0x3f, 0x3f, -1);
            sleep(1000);
            pelco.stop();
*/
class PelcoD{
	private:
	    //Variables privées
		uint8_t data[7]; // Données de commande Pelco_D 
		char sendbuffer[7]; // Buffer d'envoi de données vers le port série 
		char* portname; // Nom ou chemin du port USB du contrôle PTZ (COMX sous Windows et /dev/ttyUSBX sous Linux) 
		int baudrate; // Vitesse de comminication en Baud (9600 par défaut) 
		#ifdef __linux__ //Variables spécifiques à le version Linux
			int fd1, fd2; // Fichiers de communication USB pour Linux 
			char *buff,*buffer,*bufptr; // Buffer d'envoi 
			int wr,rd,nbytes,tries; // Variables de vérification du bon déroulement de la communication 
		#elif _WIN32 //Variables et fonctions spécifiques à la version Windows
			HANDLE h; // Handle de la connexion USB 
			void error_exit(std::string code_erreur); // Fonction d'erreur de la connexion USB 
		#endif
		//Fonctions privées
		void clear_data(); // Fonction qui nettoie les octets 3 à 7 de la commande 
		void ouvre_port_serie(); // Fonction qui ouvre la connexion série 
		void calcule_checksum(); // Fonction de calcul du dernier octet de la commande 
		void envoie_commande(); // Fonction qui envoie la commande actuelle à la caméra 
		void ferme_port_serie(); // Fonction qui ferme le port série 
	public:
		PelcoD(uint8_t adresse,char* portname);//Constructeur
		~PelcoD();//Destructeur
		//Fonctions publiques
		void pan_tilt_zoom(char vitesse_pan, char vitesse_tilt, char sens_zoom);  // Fonction qui règle la vitesse pan, tilt et zoom de la caméra 
		void retour_position_initiale(); // Fonction qui renvoie la caméra à sa position initiale 
		void focus(char sens); // Fonction qui regle le focus 
		void preset(uint8_t preset); // Fonction qui regle les preset 
		void stop(); // Fonction qui arrête la caméra 
};


#endif // ASSERVISSEMENT_PELCOD_H
