import genesis as gs

# 初期化
gs.init(backend=gs.cpu)

# シーンの作成
scene = gs.Scene(
    show_viewer=True,
    viewer_options=gs.options.ViewerOptions(
        res=(1280, 960),
        camera_pos=(3.5, 0.0, 2.5),
        camera_lookat=(0.0, 0.0, 0.5),
        camera_fov=40,
        max_FPS=60,
    ),
    vis_options=gs.options.VisOptions(
        show_world_frame=True,
        world_frame_size=1.0,
        show_link_frame=False,
        show_cameras=False,
        plane_reflection=True,
        ambient_light=(0.1, 0.1, 0.1),
    ),
    renderer=gs.renderers.Rasterizer(),
)

# シーンにエンティティを追加
plane = scene.add_entity(
    gs.morphs.Plane(),
)
franka = scene.add_entity(
    gs.morphs.MJCF(file="xml/franka_emika_panda/panda.xml"),
)
cam = scene.add_camera(
    res=(640, 480),
    pos=(3.5, 0.0, 2.5),
    lookat=(0, 0, 0.5),
    fov=30,
    GUI=False,
)

# シーンのビルド
scene.build()

# RGB・深度・セグメンテーションマスク・法線マップをレンダリング
# rgb, depth, segmentation, normal = cam.render(rgb=True, depth=True, segmentation=True, normal=True)

# カメラ録画を開始
cam.start_recording()
import numpy as np

for i in range(120):
    scene.step()

    # カメラの位置を変更
    cam.set_pose(
        pos=(3.0 * np.sin(i / 60), 3.0 * np.cos(i / 60), 2.5),
        lookat=(0, 0, 0.5),
    )
    cam.render()

# カメラ録画を停止してビデオを保存
cam.stop_recording(save_to_filename="video.mp4", fps=60)
