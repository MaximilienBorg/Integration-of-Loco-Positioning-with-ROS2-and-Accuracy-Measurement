import time
import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander

# L'adresse radio de ton Crazyflie (Par défaut)
URI = 'radio://0/30/2M/E7E7E7E7E7'

def main():
    # 1. Initialiser la clé USB radio
    cflib.crtp.init_drivers()
    print("Recherche du Crazyflie...")

    # 2. Se connecter au drone
    with SyncCrazyflie(URI) as scf:
        print("✅ Connecté au Crazyflie !")
        print("Vérification de la position LPS... Ne touchez plus le drone.")
        time.sleep(2) # Laisser le temps au LPS de se stabiliser

        # 3. Lancer le plan de vol
        # default_height=1.0 signifie que l'ordre de décollage ira à 1 mètre de haut
        with PositionHlCommander(scf, default_height=1.0) as pc:
            
            print("🚀 Étape 1 : Décollage vers (1, 1, 1)")
            # Le drone est physiquement posé en 1, 1, 0. 
            # L'ouverture du bloc 'with' déclenche le décollage automatique à Z=1.0
            time.sleep(2) # Pause de 2 secondes en l'air pour stabiliser
            
            print("➡️ Étape 2 : Vol en ligne droite vers (3, 1, 1)")
            # Vitesse par défaut : 0.2 m/s. Le drone va glisser doucement vers la cible.
            pc.go_to(3.0, 1.0, 1.0)
            time.sleep(2) # Pause de 2 secondes à l'arrivée
            
            print("🛬 Étape 3 : Atterrissage à (3, 1, 0)")
            # En quittant ce bloc de code, le drone atterrit automatiquement et coupe les moteurs.

    print("🏁 Fin de la mission, moteurs coupés.")

if __name__ == '__main__':
    main()
