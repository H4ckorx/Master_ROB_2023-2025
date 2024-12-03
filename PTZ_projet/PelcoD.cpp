#include "PelcoD.hpp"

/**
    Constructeur de la classe
    Le constructeur initialise les deux premiers octets de commande, la vitesse de communication et ouvre le port sÈrie

    @param adresse Adresse de la camÈra fixÈe avec les DIP switch
    @param portname Nom du port de communication (par exemple "COM1" sous windows et "/dev/ttyUSB0" pour Linux)
*/
PelcoD::PelcoD(uint8_t adresse,char* portname){
	this->portname=portname;
	clear_data(); //Nettoyage de la commande
	data[0] = 0xFF; //Octet de synchronisation
	data[1] = adresse; //Octet d'adresse
    baudrate = 9600; //Baudrate par dÈfaut
    ouvre_port_serie(); //Ouverture de la connection
    retour_position_initiale();
    stop();
}


/**
    Destructeur de la classe
    Le destructeur remet la camÈra en position initiale puis ferme la connexion USB
*/
PelcoD::~PelcoD(){
	std::cout << "Retour de la camera en position initiale..." << std::endl;
	retour_position_initiale();
	std::cout << "Fermeture de la connexion USB..." << std::endl;
	stop();
	ferme_port_serie();
}



/**
    Nettoyage de data
    Cette fonction remet les octets 3 ‡ 7 (data[2] ‡ data[6]) ‡ 0.
    Envoyer ‡ la camÈra une commande nettoyÈe avec clear_data() arrÍte tout mouvement de la camÈra
*/
void PelcoD::clear_data(){
	for(int i=2;i<7;i++){
        data[i]=0;
	}
}


/**
    Calcule le checksum de l'octet 7 de commande
    La fonction est privÈe car elle n'est appelÈe que dans la fonction envoie_commande()
    Le checksum est la somme des octets 2 ‡ 6 (data[1] ‡ data[5]) modulo 256
*/
void PelcoD::calcule_checksum(){
	uint16_t checksum = 0;
	for(int i = 1; i < 6; ++i){
		checksum += data[i];
	}
	checksum = checksum % 256;
	data[6] = (uint8_t)checksum;
}


/**
    RÈglage pan, tilt et zoom de la camÈra
    La fonction prend en paramËtre les vitesse pan, tilt et zoom, gÈnËre la commande et l'envoie ‡ la camÈra

    @param vitesse_pan  vitesse de rotation autour de l'axe vertical (-0x3f ‡ 0x3f)
    @param vitesse_tilt  vitesse de rotation autour de l'axe horizontal (-0x3f ‡ 0x3f)
    @param sens_zoom  zoom avant/arriËre (-1 arriËre, 1 avant, 0 pas de zoom)
*/
void PelcoD::pan_tilt_zoom(char vitesse_pan, char vitesse_tilt, char sens_zoom){
	clear_data();
	//Reglage du pan
	if(vitesse_pan < 0){ //pan gauche
		data[3] &= ~bitPanDroite;
		data[3] |= bitPanGauche;
	}else if(vitesse_pan > 0){ //pan droite
		data[3] &= ~bitPanGauche;
		data[3] |= bitPanDroite;
	}else if(vitesse_pan == 0){
		data[3] &= ~bitPanGauche;
		data[3] &= ~bitPanDroite;
	}
    vitesse_pan = abs(vitesse_pan);
    if(vitesse_pan > 0x3f){ //Limitation de la vitesse ‡ 3f (selon la doc)
		vitesse_pan = 0x3f;
	}
	data[4] = vitesse_pan;

	//Reglage du tilt
    if(vitesse_tilt < 0){ //tilt bas
		data[3] &= ~bitTiltHaut;
		data[3] |= bitTiltBas;
	}else if(vitesse_tilt > 0){ //tilt haut
		data[3] &= ~bitTiltBas;
		data[3] |= bitTiltHaut;
	}else if(vitesse_tilt == 0){
		data[3] &= ~bitTiltBas;
		data[3] &= ~bitTiltHaut;
	}
    vitesse_tilt = abs(vitesse_tilt);
    if(vitesse_tilt > 0x3f){ //Limitation de la vitesse ‡ 3f (selon la doc)
		vitesse_tilt = 0x3f;
	}
	data[5] = vitesse_tilt;

    //Reglage du zoom
    if(sens_zoom < 0){ //Zoom arriËre
		data[3] &= ~bitZoom;
		data[3] |= bitDezoom;
	}else if(sens_zoom >0){ //Zoom avant
		data[3] &= ~bitDezoom;
		data[3] |= bitZoom;
	}else if(sens_zoom == 0){ //Zoom nul
		data[3] &= ~bitDezoom;
		data[3] &= ~bitZoom;
	}

	//Envoi de la commande
	envoie_commande();
}


/**
    Retour de la camÈra en position initiale
    Cette fonction rËgle le preset qui renvoie la camÈra ‡ sa position initiale.
*/
void PelcoD::retour_position_initiale(){
    data[2] = 0;
	data[3] = 0x07;
	data[4] = 0;
	data[5] = 34;
	envoie_commande();
}



/**
    RÈglage du focus de la camÈra

    @param sens : sens du focus (-1 focus proche, 1 focus loin, 0 pas de focus)
*/
void PelcoD::focus(char sens){
	if(sens > 0){ //focus loin
		data[2] &= ~bitFocusProche;
		data[3] |= bitFocusLoin;
	}else if(sens < 0){ //focus proche
		data[3] &= ~bitFocusLoin;
		data[2] |= bitFocusProche;
	}else if(sens == 0){ //Focus nul
		data[2] &= ~bitFocusProche;
		data[3] &= ~bitFocusLoin;
	}
}



/**
    RÈglage des preset de la camÈra
    /!\ Il vaut mieux Èviter de l'utiliser pour l'instant

    @param preset : valeur du preset ‡ mettre
*/
void PelcoD::preset(uint8_t preset){
	data[2] = 0;
	data[3] = 0x07;
	data[4] = 0;
	if(preset > 0xFF){
		preset = 0xFF;
	}
	data[5] = preset;
}



/**
    ArrÍt des opÈrations de la camÈra
*/
void PelcoD::stop(){
    clear_data();
	envoie_commande();
}




#ifdef __linux__ //Version des fonctions de communication USB pour Linux_

	/**
    	Ouvre le port sÈrie.
    	Cette fonction ouvre le port sÈrie dÈfini lors de la construction de l'objet. Elle est automatiquement appelÈe lors de la construction.
	*/
	void PelcoD::ouvre_port_serie(){
		fd1=open(portname, O_RDWR | O_NOCTTY | O_NDELAY);
		if(fd1 == -1 ){
			std::cerr << "Impossible d'ouvrir le port " << portname << std::endl;
		}else{
			fcntl(fd1, F_SETFL,0);
			std::cout << "Le port " << portname << " a ete ouvert avec succes" << std::endl;
		}
	}


	/**
	    Envoie la commande de data ‡ la camÈra par le port USB
	    La fonction est privÈe car elle n'est appelÈe que dans les autres fonctions de commnde tel que pan_tilt_zoom
	*/
	void PelcoD::envoie_commande(){
		calcule_checksum();
		for(int i =0;i<7;i++){
			sendbuffer[i] = (char)data[i];
		}
		wr = write(fd1,sendbuffer,sizeof(sendbuffer));
		if(wr < 7){
			// TODO: Define a way to not call this method if the conection to the ptz camera isn't well set.
			// std::cout << "Erreur : seuls " << wr << "octets ont ÈtÈ Ècrits" << std::endl;
		}
	}

    /**
    Ferme le port sÈrie
    */
	void PelcoD::ferme_port_serie(){
		close(fd1);
	}

#elif _WIN32 //Version des fonctions de communication USB pour Windows

	/**
	    Ouvre le port sÈrie.
	    Cette fonction ouvre le port sÈrie dÈfini lors de la construction de l'objet. Elle est automatiquement appelÈe lors de la construction.
	*/
	void PelcoD::ouvre_port_serie(){
		DWORD  accessdirection = GENERIC_WRITE;
		h = CreateFile(portname,
	                    accessdirection,
	                    0,
	                    0,
	                    OPEN_EXISTING,
	                    0,
	                    0);
		if (h == INVALID_HANDLE_VALUE) {
			error_exit("CreateFile");
		}
		DCB dcbSerialParams = {0};
		dcbSerialParams.DCBlength=sizeof(dcbSerialParams);
		if (!GetCommState(h, &dcbSerialParams)) {
			 error_exit("GetCommState");
		}
		dcbSerialParams.BaudRate=baudrate;
		dcbSerialParams.ByteSize=8;
		dcbSerialParams.StopBits=ONESTOPBIT;
		dcbSerialParams.Parity=NOPARITY;
		if(!SetCommState(h, &dcbSerialParams)){
			 error_exit("SetCommState");
		}
		COMMTIMEOUTS timeouts={0};
		timeouts.ReadIntervalTimeout=50;
		timeouts.ReadTotalTimeoutConstant=50;
		timeouts.ReadTotalTimeoutMultiplier=10;
		timeouts.WriteTotalTimeoutConstant=50;
		timeouts.WriteTotalTimeoutMultiplier=10;
		if(!SetCommTimeouts(h, &timeouts)){
			error_exit("SetCommTimeouts");
		}
	}


	/**
	    GÈnËre une fenÍtre d'erreur si la connexion USB Èchoue
	    La fonction est privÈe car elle n'est appelÈe que dans la fonction ouvre_port_serie()
	*/
	void PelcoD::error_exit(std::string code_erreur){
	    // Retrieve the system error message for the last-error code
	    char *lpszFunction = &code_erreur[0u];//Conversion en char* pour utilisation
	    LPVOID lpMsgBuf;
	    LPVOID lpDisplayBuf;
	    DWORD dw = GetLastError();
	    FormatMessage(
	        FORMAT_MESSAGE_ALLOCATE_BUFFER |
	        FORMAT_MESSAGE_FROM_SYSTEM |
	        FORMAT_MESSAGE_IGNORE_INSERTS,
	        NULL,
	        dw,
	        MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
	        (LPTSTR) &lpMsgBuf,
	        0, NULL );
	    // Display the error message and exit the process
	    lpDisplayBuf = (LPVOID)LocalAlloc(LMEM_ZEROINIT,
	    (lstrlen((LPCTSTR)lpMsgBuf) + lstrlen((LPCTSTR)lpszFunction) + 40) * sizeof(TCHAR));
	    sprintf((LPTSTR)lpDisplayBuf, TEXT("%s failed with error %d:\n%s"), lpszFunction, (int)dw, (char*)lpMsgBuf);
	    MessageBox(NULL, (LPCTSTR)lpDisplayBuf, TEXT("Error"), MB_OK);
	    LocalFree(lpMsgBuf);
	    LocalFree(lpDisplayBuf);
	    ExitProcess(dw);
	}


	/**
	    Envoie la commande de data ‡ la camÈra par le port USB
	    La fonction est privÈe car elle n'est appelÈe que dans les autres fonctions de commnde tel que pan_tilt_zoom
	*/
	void PelcoD::envoie_commande(){
		calcule_checksum();
		for(int i=0;i<7;i++){
	        sendbuffer[i]=data[i];
		}
		unsigned long p=0;
		if(!WriteFile(h, sendbuffer, 7, &p, NULL)){
			error_exit("WriteFile");
		}
	}

    /**
    Ferme le port sÈrie
    */
	void PelcoD::ferme_port_serie(){
		CloseHandle(h);
	}

#endif
