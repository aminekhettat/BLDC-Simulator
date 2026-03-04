# Corrections d'Accessibilité et d'Export de Données

## Vue d'ensemble

Ce document détaille les corrections apportées pour résoudre les deux problèmes principaux :

1. **Problème d'accessibilité** : Navigation impossible dans les onglets et listes avec un lecteur d'écran
2. **Problème d'export** : Les fichiers de données n'étaient pas enregistrés au bon endroit

## Problèmes résolus

### 1. ✅ Exportation de Données Non Fonctionnelle

#### Problème

- L'utilisateur choisissait un emplacement d'export via `QFileDialog.getSaveFileName()`
- Le chemin sélectionné était **ignoré**
- Les fichiers étaient toujours enregistrés dans le répertoire `LOGS_DIR` par défaut
- L'utilisateur n'avait aucun contrôle sur la destination

#### Solution

**Fichier modifié:** [src/utils/data_logger.py](src/utils/data_logger.py)

✨ **Changements:**

- Ajout d'un paramètre `use_custom_path` à la méthode `save_simulation_data()`
- Logique améliorée pour traiter les chemins personnalisés:
  ```python
  if use_custom_path:
      # Use full path provided by user
      csv_file = Path(filename)
      if csv_file.suffix != ".csv":
          csv_file = csv_file.with_suffix(".csv")
      # Create metadata file with same name but _metadata suffix
      json_file = csv_file.parent / f"{csv_file.stem}_metadata.json"
      csv_file.parent.mkdir(parents=True, exist_ok=True)
  ```

**Fichier modifié:** [src/ui/main_window.py](src/ui/main_window.py#L1246-L1276)

✨ **Changements:**

- Mise à jour de `_export_data()` pour utiliser le chemin personnalisé de l'utilisateur
- Ajout de gestion d'erreurs pour les échecs d'export
- Notification de l'utilisateur avec feedback audio

#### Resultat

- ✅ Les fichiers sont maintenant enregistrés au chemin choisi par l'utilisateur
- ✅ Les fichiers métadonnées sont créés avec le même préfixe
- ✅ Les répertoires sont créés automatiquement si nécessaire

---

### 2. ✅ Accessibilité des Onglets et Listes

#### Problème

- Les listes de sélection (variables de monitoring, variables à afficher) n'étaient pas accessibles au clavier
- Pas de support pour la navigation avec un lecteur d'écran
- Les utilisateurs de lecteurs d'écran ne pouvaient pas:
  - Naviguer dans les onglets
  - Accéder à la liste de monitoring
  - Sélectionner des variables pour les graphiques

#### Solution

**Fichier modifié:** [src/ui/widgets/accessible_widgets.py](src/ui/widgets/accessible_widgets.py)

✨ **Nouvelles classes créées:**

1. **`AccessibleTableWidget`** - Table accessible avec:
   - ✅ Noms et descriptions ARIA pour lecteurs d'écran
   - ✅ Navigation au clavier avec touches fléchées (↑↓←→)
   - ✅ Sélection avec Space ou Enter
   - ✅ Annonce du contenu des cellules et de la sélection
   - ✅ Support du focus au clavier

2. **`AccessibleListWidget`** - Liste accessible avec:
   - ✅ Noms et descriptions ARIA
   - ✅ Navigation au clavier avec flèches (↑↓)
   - ✅ Sélection avec Space ou Enter
   - ✅ Position dans la liste annoncée

**Fichier modifié:** [src/ui/main_window.py](src/ui/main_window.py)

✨ **Améliorations:**

1. **Onglet Monitoring** - Nouvelle structure accessible:
   - Instructions de navigation visibles et annoncées
   - Liste de monitoring focusable avec descriptions détaillées
   - Chaque valeur peut être navigable au clavier
   - Graphique de vitesse avec description accessible

2. **Onglet Plotting** - Sélecteur de variables amélioré:
   - Remplacement de `QTableWidget` par `AccessibleTableWidget`
   - Support complet pour lecteur d'écran
   - Navigation au clavier avec flèches
   - Sélection/désélection avec Space/Enter

#### Résultats

- ✅ Navigation complète des onglets avec Tab et Ctrl+Tab
- ✅ Les utilisateurs de lecteurs d'écran peuvent naviguer dans les listes
- ✅ Accès au clavier à tous les contrôles
- ✅ Feedback auditif approprié pour les actions

---

## Navigation au Clavier

### Onglets

- **Tab** : Naviguer vers le widget suivant
- **Shift+Tab** : Naviguer vers le widget précédent
- **Ctrl+Tab** : Passer à l'onglet suivant

### Listes et Tables

- **↑↓** : Naviguer entre les éléments
- **←→** : Naviguer entre les colonnes (tables)
- **Space / Enter** : Sélectionner/Désélectionner
- **Tab** : Sortir de la liste et aller au widget suivant

### Lecteur d'écran

- Les noms accessibles (`AccessibleName`) décrivent le widget
- Les descriptions (`AccessibleDescription`) expliquent l'utilisation
- Les tooltips affichent les informations de sélection
- Les indices de position sont annoncés (ex: "Item 1 of 5")

---

## Fichiers Modifiés

### 1. [src/utils/data_logger.py](src/utils/data_logger.py)

- ✅ Ajout du paramètre `use_custom_path` à `save_simulation_data()`
- ✅ Logique améliorée pour les téléchargements personnalisés
- ✅ Création automatique de répertoires

### 2. [src/ui/widgets/accessible_widgets.py](src/ui/widgets/accessible_widgets.py)

- ✅ Nouvelles classes `AccessibleTableWidget` et `AccessibleListWidget`
- ✅ Gestion du clavier améliorée
- ✅ Support complet du lecteur d'écran

### 3. [src/ui/main_window.py](src/ui/main_window.py)

- ✅ Import et utilisation de `AccessibleTableWidget`
- ✅ Amélioration de `_create_monitoring_tab()` avec instructions de navigation
- ✅ Amélioration de `_export_data()` avec support du chemin personnalisé
- ✅ Ajout de descriptions accessibles détaillées

---

## Tests de Validation

Tous les tests réussissent ✅ :

```
✅ PASS: Imports
✅ PASS: DataLogger Custom Paths
✅ PASS: Accessible Widgets
```

Exécuter les tests:

```bash
python test_accessibility_fixes.py
```

---

## Notes Utilisateur

### Pour Exporter les Données

1. Lancez une simulation
2. Cliquez sur "Export Data" ou Ctrl+S
3. Choisissez l'emplacement et le nom du fichier
4. ✅ Les fichiers sont enregistrés au chemin choisi

### Pour Utiliser avec un Lecteur d'Écran

1. Utilisez Tab pour naviguer entre les onglets
2. Dans chaque onglet, utilisez Tab ou les flèches pour naviguer
3. Écoutez les descriptions annoncées par le lecteur
4. Utilisez Space/Enter pour sélectionner les éléments
5. Les paramètres sont annoncés lors du changement

### Raccourcis Clavier Disponibles

- **F5** : Démarrer la simulation
- **F6** : Arrêter la simulation
- **F7** : Réinitialiser la simulation
- **Ctrl+S** : Exporter les données
- **Ctrl+Q** : Quitter l'application

---

## Améliorations Futures Possibles

- [ ] Support des touches de pagination (Page Up/Down) dans les listes
- [ ] Mode focus amélioré pour les éléments de liste
- [ ] Navigation avec j/k (Vim-like) optionnelle
- [ ] Annonces vocales personnalisées plus détaillées

---

**Version:** 2.0.0 - Modifications d'Accessibilité
**Date:** Octobre 2024
**Auteur:** BLDC Control Team - Accessibility Update
