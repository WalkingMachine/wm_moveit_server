# wm_moveit_server

## description
Simple interface permettant de faire des appels de services à un move_group.

## services offerts

- ### move
  bouge un movegroup à une pose donné.

- ### move_joints
  bouge un movegroup à un joint_state prédéfinit.
  
- ### get_gose
  donne la pose actuelle d'un movegroup.

## dependences
- [sara_moveit](https://github.com/WalkingMachine/sara_moveit)

- [agile grasp](https://github.com/WalkingMachine/agile_grasp)(pas pour l'instant)

- [grasp_selection](https://github.com/atenpas/grasp_selection)(pas pour l'instant)

## utilisation
1- lancer le move_group

2- lancer wm_moveit_server

3- faite des appels de services
