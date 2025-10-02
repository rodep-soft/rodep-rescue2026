# rodep-rescue2026

## Branch rules

- mainへのpushは不可能
- 作業時は必ずbranchを切って、PRを出すこと
- ciが通っていないのにmergeしない

## Directory explanation

- ros_ws -> メインのROS2 workspace
  - src -> nodeのpkgを配置
  - sandbox -> 試験的なコード(特にルールはなし)
- scripts -> モーターの単体テストやbash script
- docs -> Linuxのsetupや書き留
