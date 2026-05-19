import math
import time
import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander

# /!\ REMPLACE CECI PAR LA VRAIE ADRESSE DE TON DRONE
URI = 'radio://0/30/2M/E7E7E7E7E7'

def main():
    # Initialisation de la clé USB radio
    cflib.crtp.init_drivers()
    print("Recherche du Crazyflie...")

    with SyncCrazyflie(URI) as scf:
        print("✅ Connecté ! Ne touchez plus le drone pendant 3 secondes.")
        time.sleep(3)

        print("🚀 Décollage depuis le centre (2, 2, 0)...")
        # On décolle à 1 mètre d'altitude
        with PositionHlCommander(scf, default_height=1.0) as pc:
            time.sleep(2) # Pause de stabilisation

            print("➡️ Déplacement vers le bord du cercle (3, 2, 1)...")
            pc.go_to(3.0, 2.0, 1.0)
            time.sleep(2)

            print("🔄 Début de la chorégraphie (3 cercles)...")
            center_x = 2.0
            center_y = 2.0
            radius = 1.0
            altitude = 1.0

            # On fait 3 tours (3 fois 360 degrés = 1080 degrés)
            for degree in range(360 * 3):
                # Conversion des degrés en radians pour les maths
                rad = math.radians(degree)
                
                # Calcul de la position X et Y sur le cercle
                x = center_x + radius * math.cos(rad)
                y = center_y + radius * math.sin(rad)

                # Envoi de la position fluide au drone (sans le faire s'arrêter)
                scf.cf.commander.send_position_setpoint(x, y, altitude, 0)
                
                # Cette pause définit la VITESSE du drone. 
                # 0.04 seconde par degré = Environ 14 secondes pour faire un tour complet. C'est très doux !
                time.sleep(0.04)

            # Fin de la boucle, le drone a terminé ses cercles et se trouve en (3, 2, 1)
            print("⬅️ Retour au centre de la pièce (2, 2, 1)...")
            pc.go_to(2.0, 2.0, 1.0)
            time.sleep(2)

            print("🛬 Atterrissage...")
            # L'atterrissage se fait tout seul en sortant du bloc 'with'

    print("🏁 Mission terminée, moteurs coupés.")

if __name__ == '__main__':
    main()
