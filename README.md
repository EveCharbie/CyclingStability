# CyclingStability

## Jules' todo list
- Equation du mouvement du modèle le plus simple de SimBRiM
- Paramètres d'un vélo basique
- Proposer des conditions de validation du modèle

## Validation du modèle

Le modèle peut être dit predictif/realiste à certains niveaux si il remplit certains des critères suivants:
- VALIDATION MOYENNE produits des quantités angulaires (ex: amplitudes des vitesses angulaires, frequences) conformes aux mesures experimentales dans des conditions d'observations similaires (pertubations, suivi de ligne, maintien de l'equilibre) -> Extraire les donnees de ref chez Moore et al., Dialynas at al., Ronné et al.
- VALIDATION MOYENNE+ produit un mouvement d'une stabilite (variations du moment cinetique, temps retour a l'equilibre, cycle limite) proche de ceux observes experimentalement (pertubations, suivi de ligne, maintien de l'equilibre) ->Extraire les donnees de ref chez Moore et al. voire perturbations + chute
- VALIDATION FAIBLE produit une strategie de maintient de l'equilibre temporelle qualitativement similaire (braquage, contre braquage, lean, counter lean) -> On manque de contruit sur cet aspect qualitatif, ca ira pas loin
- VALIDATION FORTE produit une performance de tâche reproduisant les effets: du design du velo, de la vitesse -> Donnees de Ronné et al.
- produit une stategie visuelle compatible/similaire avec mes dernieres observations
- VALIDATION FAIBLE, produit un comportement fréquentiel vérifiant la théorie de McRuer et Jex
En cas de validation de certains de ces niveaux il est possible

| Nom                  | Niveau    | Description                                                                                                    | Métriques/Analyses              | Ce que ça demande de faire                                      | Intérêt                                                   | Ref                     |
|----------------------|-----------|----------------------------------------------------------------------------------------------------------------|----------------------------------|---------------------------------------------------------------|-----------------------------------------------------------|-------------------------|
| Stratégies qualitatives | FAIBLE    | Stratégie de maintien de l'équilibre temporelle similaire (braquage, contre-braquage, lean, counter lean)       | Séquences d'actions             | Codifier les mouvements en séquences (jamais fait) + décrire des séquences expé (notre ref) | On manque de construit sur cet aspect qualitatif, ça ira pas loin |                         |
| Cinématique   | MOYENNE   | Produit des quantités angulaires (ex: amplitudes des vitesses angulaires, fréquences) conformes aux mesures expérimentales dans des conditions d'observations similaires (perturbations, suivi de ligne, maintien de l'équilibre) | Petit traitement du signal de données open source              | Extraire les données de ref chez Moore et al., Dialynas et al., Ronné et al. |      | Moore et al., Dialynas et al., Ronné et al. |
|Dynamique / Stabilité  | MOYENNE+  | Produit un mouvement d'une stabilité proche de ceux observés expérimentalement (perturbations, suivi de ligne, maintien de l'équilibre) | moment cinétique, temps retour à l'équilibre, cycle limite    | Extraire les données de ref chez Moore et al. (voire perturbations + chute) |                   | Moore et al. + autre         |
| Reproduction des perfs     | FORTE     | Produit une performance de tâche reproduisant les effets : du design du vélo, de la vitesse                     | distance à la ligne                | Un scénario de suivi de ligne                                      |  | Ronné et al.            |
| Stratégie visuelle   | FAIBLE    | Produit une stratégie visuelle compatible/similaire avec les dernières observations                            | Métriques a définir            | Inclure le mouvement des yeux dans le modèle                           |        |                         |
| Domaine fréquentiel | FAIBLE    | Produit un comportement fréquentiel vérifiant la théorie de McRuer et Jex                                      | Diagramme de Bode           | Tester le modèle a differentes fréquences de commande    |  | McRuer et Jex           |




## Ressources numériques

- https://moorepants.github.io/learn-multibody-dynamics/nonholonomic-eom.html
- https://mechmotum.github.io/symbrim/tutorials/my_first_bicycle.html
