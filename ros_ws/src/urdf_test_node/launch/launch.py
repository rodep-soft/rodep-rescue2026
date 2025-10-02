import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution


def generate_launch_description():
    pkg_name = "urdf_test_node"
    # 🚨 修正点1: ファイル名が 'robot.urdf.xacro' で、'urdf' サブディレクトリにあると仮定
    robot_file_name = "robot.urdf.xacro"

    pkg_share_dir = get_package_share_directory(pkg_name)

    # 1. URDFファイルの完全なパスを構築
    # Launchファイルが想定しているパス: share/urdf_test_node/urdf/robot.urdf.xacro
    urdf_path = os.path.join(pkg_share_dir, "urdf", robot_file_name)

    # 2. XACROファイルを読み込み、URDFに展開する (Command サブスティチューションを使用)
    #    os.popen() の代わりに Command() を使うことで、Launchの実行環境内で安全に処理
    robot_description_content = Command(
        [
            PathJoinSubstitution(
                [FindExecutable(name="xacro")]
            ),  # xacro コマンドのパスを探す
            " ",
            urdf_path,
        ]
    )

    # 3. robot_state_publisher ノードの定義
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content}
        ],  # Commandの出力を使用
    )

    # 4. RVIz2 ノードの定義
    rviz_config_path = os.path.join(pkg_share_dir, "rviz", "config.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz_node",
        output="screen",
        arguments=["-d", rviz_config_path],
        # 🚨 修正点2: RSPノードが起動してからRvizを起動するよう依存関係を設定 (必須ではないが推奨)
        # depends_on=[rsp_node],
    )

    return LaunchDescription(
        [
            rsp_node,
            rviz_node,
        ]
    )
