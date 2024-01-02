#include <GL/freeglut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <stdbool.h> // Introduction du support de type booléen
#include <time.h> // Inclut la déclaration de la fonction time
#include <stdlib.h> // Inclut les déclarations des fonctions srand et rand
#include <stdio.h> // Inclut la déclaration de la fonction sprintf

/*********************************************************************** Paramètres de base *****************************************************************************/

// Définition des dimensions de la zone de jeu et de la zone périphérique
const int gameAreaWidth = 600;  // Largeur de la zone de jeu
const int gameAreaHeight = 800; // Hauteur de la zone de jeu
const int borderSize = 100;     // Taille de la bordure

// Dimensions totales de la fenêtre
const int windowWidth = gameAreaWidth + 2 * borderSize;
const int windowHeight = gameAreaHeight + 2 * borderSize;

// Définition de la taille du plateau de jeu
#define ROWS 8
#define COLS 6

// Structure de données du plateau de jeu
int gameBoard[ROWS][COLS] = {{0}};

// Intervalle de temps pour générer de nouveaux blocs (en millisecondes)
#define GENERATE_INTERVAL 10000
// Variable du compte à rebours, en millisecondes
float countdown = 100000;

// Taille des blocs
#define BLOCK_SIZE 92

// Définition de la taille de l'espace entre les grilles
#define GAP_SIZE 7.5

// Fonction d'initialisation
void init() {
    glClearColor(0.0, 0.0, 0.0, 1.0); // Définit la couleur de fond
}

// Etat du démarrage du jeu
bool gameStarted = false;
bool showTimerBar = false;

// Configuration des couleurs
float colors[20][3] = {
    {1.0, 0.0, 0.0},   // Rouge
    {0.0, 1.0, 0.0},   // Vert
    {0.0, 0.0, 1.0},   // Bleu
    {1.0, 1.0, 0.0},   // Jaune
    {1.0, 0.0, 1.0},   // Violet
    {0.0, 1.0, 1.0},   // Cyan
    {1.0, 1.0, 1.0},   // Blanc
    {0.0, 0.0, 0.0},   // Noir
    {1.0, 0.5, 0.0},   // Orange
    {1.0, 0.75, 0.8},  // Rose
    {0.0, 0.0, 0.55},  // Bleu foncé
    {0.5, 0.5, 0.0},   // Vert olive
    {0.6, 0.4, 0.12},  // Marron
    {0.5, 0.5, 0.5},   // Gris
    {0.53, 0.81, 0.98},// Bleu clair
    {0.55, 0.0, 0.0},  // Rouge foncé
    {0.0, 0.39, 0.0},  // Vert foncé
    {0.58, 0.0, 0.82}, // Violet
    {0.68, 0.85, 0.9}, // Bleu pâle
    {1.0, 0.84, 0.0}   // Or
};

bool isDragging = false; // Indique si un bloc est en train d'être déplacé
int draggedBlockX = -1;  // Colonne du bloc déplacé
int draggedBlockY = -1;  // Ligne du bloc déplacé
int mouseX = 0, mouseY = 0; // Coordonnées de la souris à l'écran

/*************************************************************** Mise à jour de l'état du jeu ******************************************************************/

/* Obtenir le score maximal */
int getMaxScore() {
    int maxScore = 0;
    for (int i = 0; i < ROWS; i++) {
        for (int j = 0; j < COLS; j++) {
            if (gameBoard[i][j] > maxScore) {
                maxScore = gameBoard[i][j];
            }
        }
    }
    return maxScore;
}

/* Déplacer les blocs vers le haut */
void shiftBlocksUp() {
    for (int i = ROWS-1; i >= 0; i--) {
        for (int j = 0; j < COLS; j++) {
            gameBoard[i][j] = gameBoard[i-1][j];
        }
    }
    for (int j = 0; j < COLS; j++) {
           gameBoard[0][j] = 0;
       }
}

/* Générer un nouveau bloc aléatoire sans reproduire le bloc supérieur */
int generateRandomBlock(int maxVal, int exclude) {
    int newVal;
    do {
        newVal = rand() % maxVal + 1;
    } while (newVal == exclude);
    return newVal;
}

/* Générer une nouvelle ligne de blocs */
void generateNewRow() {
    shiftBlocksUp(); // Déplace les blocs existants vers le haut

    int maxScore = getMaxScore(); // Obtenir le score maximal actuel

    for (int j = 0; j < COLS; j++) {
        int exclude = (ROWS > 1) ? gameBoard[1][j] : 0; // Exclure la valeur de la deuxième ligne de la colonne actuelle
        gameBoard[0][j] = generateRandomBlock(maxScore - 2, exclude); // Générer un nouveau bloc
    }
}

/* Initialiser le jeu */
void initializeGame() {
    gameStarted = true;
    showTimerBar = true; // Afficher la barre de progression
    countdown = GENERATE_INTERVAL; // Initialiser le compte à rebours

    // Vider le plateau de jeu
    for (int i = 0; i < ROWS; i++) {
        for (int j = 0; j < COLS; j++) {
            gameBoard[i][j] = 0;
        }
    }
    // Générer deux lignes de blocs aléatoires, en veillant à ce que les blocs supérieurs et inférieurs ne soient pas identiques
    int maxVal = 6; // Supposons que la valeur maximale du bloc soit 6
    for (int j = 0; j < COLS; j++) {
        gameBoard[1][j] = rand() % maxVal + 1; // Générer un bloc aléatoire pour l'avant-dernière ligne
        gameBoard[0][j] = generateRandomBlock(maxVal, gameBoard[1][j]); // Générer un bloc pour la dernière ligne
    }
}

/* Gestion des événements de la souris */
void mouse(int button, int state, int x, int y) {
    int transformedY = windowHeight - y;

    // Vérifier si le jeu a commencé
    if (!gameStarted) {
        // Si le bouton gauche de la souris est pressé
        if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
            // Vérifier si le bouton de démarrage est cliqué
            if (x >= 300 && x <= 500 && transformedY >= 450 && transformedY <= 550) {
                gameStarted = true;
                countdown = 0;
                initializeGame();
            }
        }
    } else {
        // Gérer les événements de la souris pendant le jeu
        if (button == GLUT_LEFT_BUTTON) {
            if (state == GLUT_DOWN) {
                // Commencer à glisser un bloc
                draggedBlockX = (x - borderSize) / (BLOCK_SIZE + GAP_SIZE);
                draggedBlockY = (transformedY - borderSize) / (BLOCK_SIZE + GAP_SIZE);
                if (draggedBlockX >= 0 && draggedBlockX < COLS && draggedBlockY >= 0 && draggedBlockY < ROWS) {
                    isDragging = true;
                }
            } else if (state == GLUT_UP && isDragging) {
                // Arrêter de glisser
                isDragging = false;

                // Calculer la position de dépôt
                int dropX = (x - borderSize) / (BLOCK_SIZE + GAP_SIZE);
                int dropY = (transformedY - borderSize) / (BLOCK_SIZE + GAP_SIZE);

                // Gérer le dépôt du bloc
                if (dropX >= 0 && dropX < COLS && dropY >= 0 && dropY < ROWS) {
                    int draggedValue = gameBoard[draggedBlockY][draggedBlockX];
                    int dropValue = gameBoard[dropY][dropX];

                    // Placer ou fusionner les blocs
                    if (dropValue == 0 || draggedValue == dropValue) {
                        gameBoard[dropY][dropX] = (dropValue == 0) ? draggedValue : dropValue + 1;
                        gameBoard[draggedBlockY][draggedBlockX] = 0;
                    }
                }

                // Réinitialiser l'état de glissement
                draggedBlockX = -1;
                draggedBlockY = -1;
            }
        }
    }
}

void motion(int x, int y) {
    // Mise à jour de la position de la souris
    mouseX = x;
    mouseY = windowHeight - y;
    glutPostRedisplay(); // Demander le redessin
    printf("Position de la souris : (%d, %d)\n", mouseX, mouseY);
}

void keyboard(unsigned char key, int x, int y) {
    // Quitter le jeu avec la touche ESC
    if (key == 27) {
        exit(0);
    }
}

void updateGame() {
    // Vérifier si des blocs atteignent le haut
    for (int j = 0; j < COLS; j++) {
        if (gameBoard[0][j] != 0) {
            // Logique de fin de jeu
            break;
        }
    }

    glutPostRedisplay();
}

void mergeBlocks(int fromRow, int fromCol, int toRow, int toCol) {
    // Logique de fusion de deux blocs
}

void removeCompleteRows() {
    // Vérifier et éliminer les lignes complètes
}

void onTimer(int value) {
    if (!gameStarted) {
        countdown = 0;
        glutTimerFunc(10000, onTimer, 0);
        return; // Retourner si le jeu n'a pas commencé
    }
    countdown -= 10000; // Réduire le compte à rebours

    if (countdown <= 0) {
        generateNewRow();  // Générer une nouvelle ligne de blocs
        updateGame();      // Mettre à jour l'état du jeu
        countdown = GENERATE_INTERVAL; // Réinitialiser le compte à rebours
    }

    glutPostRedisplay();
    glutTimerFunc(10000, onTimer, 0);
}

void updateTimerBar(int value) {
    if (!gameStarted) {
        countdown = 0;
        glutTimerFunc(20, updateTimerBar, 0);
        return; // Retourner si le jeu n'a pas commencé
    }
    countdown -= 20;
    if (countdown <= 0) {
        countdown = GENERATE_INTERVAL; // Réinitialiser le compte à rebours
    }

    glutPostRedisplay();
    glutTimerFunc(20, updateTimerBar, 0); // Appel toutes les 20 millisecondes
}

void drawSquare(float x, float y, float size) {
    // Dessiner un carré
    glBegin(GL_QUADS);
        glVertex2f(x, y);
        glVertex2f(x + size, y);
        glVertex2f(x + size, y + size);
        glVertex2f(x, y + size);
    glEnd();
}

void drawText(const char *text, float x, float y) {
    // Dessiner du texte
    glRasterPos2f(x, y);
    for (const char *c = text; *c != '\0'; c++) {
        glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, *c);
    }
}

void drawBlockAtScreenPosition(int value, int screenX, int screenY) {
    // Dessiner un bloc à la position de l'écran
    float x = screenX - BLOCK_SIZE / 2.0f;
    float y = screenY - BLOCK_SIZE / 2.0f;

    // Choisir la couleur du bloc
    if (value > 0 && value <= 20) {
        glColor3f(colors[value-1][0], colors[value-1][1], colors[value-1][2]);
    } else {
        glColor3f(0.5, 0.5, 0.5); // Couleur par défaut
    }

    drawSquare(x, y, BLOCK_SIZE); // Dessiner le bloc

    // Définir la couleur du texte (par exemple noir)
    glColor3f(0.0, 0.0, 0.0);
    char numStr[3];
    sprintf(numStr, "%d", value);
    drawText(numStr, x + (BLOCK_SIZE / 2.2), y + (BLOCK_SIZE / 2.2));
}

void drawStartButton() {
    // Dessiner le bouton de démarrage
    glColor3f(0.968627f, 0.862745f, 0.435294f); // Bouton couleur crème
    glBegin(GL_QUADS);
        glVertex2f(300, 450);
        glVertex2f(500, 450);
        glVertex2f(500, 550);
        glVertex2f(300, 550);
    glEnd();

    // Définir la couleur du texte et dessiner le texte sur le bouton
    glColor3f(0.490196f, 0.235294f, 0.596078f); // Texte violet riche
    drawText("Commencer", 345, 485); // Supposons que le texte est au milieu du bouton

    // Définir la couleur gris foncé et dessiner les instructions du jeu au-dessus du bouton
    glColor3f(0.3f, 0.3f, 0.3f); // Texte gris foncé
    drawText("Bienvenue dans Twenty!", 300, 700); // Titre
    drawText("Pour jouer, prenez des tuiles et déposez-les sur des tuiles", 200, 620); // Instruction partie 1
    drawText("de même valeur", 300, 560); // Instruction partie 2
}
