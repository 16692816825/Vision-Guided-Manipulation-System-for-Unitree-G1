# ui/styles.py

class Styles:
    # =========================================================
    # 1. 全局主窗口样式 (原 setup_style)
    # =========================================================
    GLOBAL = """
        /* 全局基础设置 */
        QMainWindow { background-color: #1e1e1e; color: #e0e0e0; }
        QWidget { font-family: "Microsoft YaHei UI", "Segoe UI"; font-size: 14px; color: #e0e0e0; }
        
        /* 面板样式 */
        QFrame#Panel { background-color: #2d2d2d; border-radius: 8px; border: 1px solid #3d3d3d; }
        
        /* 标题样式 */
        QLabel#Title { font-size: 22px; font-weight: bold; color: #00e5ff; }
        QLabel#SubTitle { font-size: 14px; font-weight: bold; color: #aaa; border-bottom: 1px solid #444; margin-top: 10px; padding-bottom: 5px;}
        
        /* 文本框和进度条 */
        QTextEdit { background-color: #111; border: 1px solid #333; color: #00ff00; font-family: "Consolas"; font-size: 12px; }
        QProgressBar { border: none; background-color: #111; height: 6px; border-radius: 3px; }
        QProgressBar::chunk { background-color: #00e5ff; border-radius: 3px; }

        /* =======================================================
           Tab 选项卡样式优化 - 扁平化 + 底部高亮 + 暗色滚动条
           ======================================================= */
        QTabWidget::pane { 
            border: 1px solid #444; 
            background: #1e1e1e; /* 内容区背景 */
            top: -1px; /* 消除边框重叠 */
        }

        QTabBar::tab { 
            background: #2d2d2d; /* 未选中背景：稍亮一点的灰 */
            color: #888;         /* 未选中文字：暗灰 */
            padding: 10px 15px;  /* 增加内边距，让点击区域更大 */
            border: none;
            border-bottom: 2px solid transparent; /* 预留底部线条空间 */
            margin-right: 1px;   /* Tab 之间的间隔 */
            font-weight: bold;
            font-size: 13px;
        }

        /* 选中状态：背景变深（融入内容），文字变青，底部出现青色线条 */
        QTabBar::tab:selected { 
            background: #1e1e1e; 
            color: #00e5ff; 
            border-bottom: 2px solid #00e5ff; 
        }

        /* 鼠标悬停状态 */
        QTabBar::tab:hover:!selected {
            background: #333;
            color: #ccc;
        }

        /* --- 修复那个丑陋的白色滚动按钮 --- */
        QTabBar::scroller {
            width: 24px; /* 设置按钮宽度 */
        }

        /* 定义滚动箭头按钮的样式 */
        QTabBar QToolButton {
            border: none;
            background-color: #2d2d2d; /* 设为深色背景 */
            color: #00e5ff;            /* 箭头颜色设为青色 */
            border-radius: 0px;
        }

        QTabBar QToolButton:hover {
            background-color: #383838; /* 悬停变亮 */
        }
        
        /* 表格样式 */
        QTableWidget { background-color: #1a1a1a; gridline-color: #333; border: none; font-family: Consolas; font-size: 12px; }
        QHeaderView::section { background-color: #333; color: #fff; padding: 4px; border: none; }
        
        /* 弹窗样式 */
        QMessageBox { background-color: #2d2d2d; color: #ffffff; }
        QMessageBox QLabel { color: #ffffff; background-color: transparent; }
        QMessageBox QPushButton {
            background-color: #444; color: white; border: 1px solid #666;
            border-radius: 4px; padding: 5px 15px; min-width: 60px;
        }
        QMessageBox QPushButton:hover { background-color: #555; }
        
        /* 输入框弹窗样式 */
        QInputDialog { background-color: #2d2d2d; color: #ffffff; }
        QInputDialog QLabel { color: #ffffff; background-color: transparent; }
        QInputDialog QLineEdit {
            background-color: #1a1a1a; color: #e0e0e0;
            border: 1px solid #444; border-radius: 4px; padding: 4px;
        }
        QInputDialog QPushButton {
            background-color: #444; color: white; border: 1px solid #666;
            border-radius: 4px; padding: 5px 15px; min-width: 60px;
        }
        QInputDialog QPushButton:hover { background-color: #555; }
    """

    # =========================================================
    # 2. Tab 1: 数据监控 (Monitor)
    # =========================================================
    MONITOR_CARD = """
        QFrame#Card {
            background-color: #1e1e1e;
            border: 1px solid #333;
            border-radius: 8px;
        }
        QLabel#CardTitle {
            color: #888;
            font-size: 12px;
            font-weight: bold;
            font-family: 'Microsoft YaHei UI';
            letter-spacing: 1px;
            background-color: transparent;
            padding: 5px;
        }
        QLabel#ValBig {
            font-family: 'Consolas', 'Arial'; 
            font-size: 26px; 
            font-weight: bold;
        }
        QLabel#ValNorm {
            font-family: 'Consolas', 'Arial'; 
            font-size: 18px; 
            font-weight: bold;
        }
        QLabel#Unit { color: #555; font-size: 10px; margin-top: 8px; }
    """

    MONITOR_PROGRESS_BAR = """
        QProgressBar {
            border: 1px solid #333;
            background-color: #111;
            border-radius: 4px;
        }
        QProgressBar::chunk {
            /* 渐变色：绿 -> 黄 -> 红 */
            background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #00e676, stop:0.5 #ffeb3b, stop:1 #ff5252);
            border-radius: 4px;
        }
    """

    # =========================================================
    # 3. Tab 2: 关节详情 (Joints)
    # =========================================================
    JOINTS_TABLE = "QTableWidget { background-color: #1a1a1a; alternate-background-color: #252525; gridline-color: #333; } QHeaderView::section { background: #333; color: white; }"

    # =========================================================
    # 4. Tab 3: 示教模式 (Teach)
    # =========================================================
    TEACH_GROUP = """
        QGroupBox {
            border: 1px solid #444;
            border-radius: 6px;
            margin-top: 10px;
            background-color: #252525;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 5px;
            color: #aaa;
            font-weight: bold;
            font-size: 11px;
        }
        QLabel { color: #ccc; font-family: "Microsoft YaHei UI"; }
    """

    TEACH_LIST = """
        QListWidget { 
            background: #1a1a1a; 
            border: 1px solid #444; 
            font-size: 14px; 
            color: #e0e0e0;
            padding: 5px;
        }
        QListWidget::item { 
            padding: 8px; 
            border-bottom: 1px solid #2d2d2d; 
        }
        QListWidget::item:selected { 
            background-color: #6200ea; 
            color: white; 
            border: 1px solid #7c4dff;
            border-radius: 4px;
        }
        QListWidget::item:hover:!selected {
            background-color: #2d2d2d;
        }
    """

    BTN_RECORD_ORANGE = """
        QPushButton { 
            background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #e65100, stop:1 #ff6f00); 
            color: white; font-weight: bold; font-size: 16px; border-radius: 8px; border: 1px solid #e65100;
        }
        QPushButton:hover { background-color: #ff8f00; }
        QPushButton:pressed { border: 2px solid white; }
    """

    BTN_REPLAY_BLUE = """
        QPushButton { 
            background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #0277bd, stop:1 #039be5); 
            color: white; font-weight: bold; font-size: 16px; border-radius: 8px; border: 1px solid #0277bd;
        }
        QPushButton:hover { background-color: #29b6f6; }
    """

    BTN_EXIT_TEACH = """
        QPushButton { 
            background-color: #37474f; color: #78909c; font-weight: bold; font-size: 14px; border-radius: 6px; border: 1px solid #455a64;
        }
        QPushButton:enabled {
            background-color: #b71c1c; color: white; border: 1px solid #e57373;
        }
        QPushButton:enabled:hover { background-color: #c62828; }
    """

    # =========================================================
    # 5. Tab 4: 精确控制 (Precision)
    # =========================================================
    PRECISION_GROUP = """
        QGroupBox {
            border: 1px solid #444; border-radius: 6px; margin-top: 10px; background-color: #252525;
        }
        QGroupBox::title {
            subcontrol-origin: margin; left: 10px; padding: 0 5px; color: #aaa; font-weight: bold; font-size: 11px;
        }
        QLabel { color: #ccc; font-size: 12px; font-family: "Microsoft YaHei UI"; }
        QLineEdit { 
            background-color: #1a1a1a; color: #00e5ff; border: 1px solid #333; 
            border-radius: 4px; padding: 4px; font-family: Consolas; font-weight: bold; 
        }
    """

    BTN_AUTO_GRASP = """
        QPushButton { 
            background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #d500f9, stop:1 #aa00ff);
            color: white; font-weight: bold; font-size: 15px; 
            border-radius: 6px; border: 1px solid #ea80fc;
        }
        QPushButton:hover { background-color: #e040fb; border: 2px solid white;}
        QPushButton:pressed { background-color: #4a148c; }
    """

    BTN_IK_EXECUTE = """
        QPushButton { 
            background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #6200ea, stop:1 #7c4dff); 
            color: white; font-weight: bold; font-size: 14px; border-radius: 4px; border: 1px solid #6200ea;
        }
        QPushButton:hover { background-color: #651fff; }
    """

    @staticmethod
    def get_slider_style(color_hex):
        """动态生成带颜色的滑块样式"""
        return f"""
            QSlider::groove:horizontal {{
                border: 1px solid #333;
                height: 6px;
                background: #222;
                border-radius: 3px;
            }}
            QSlider::sub-page:horizontal {{
                background: {color_hex};
                border-radius: 3px;
            }}
            QSlider::handle:horizontal {{
                background: #fff;
                border: 1px solid {color_hex};
                width: 14px;
                height: 14px;
                margin: -5px 0;
                border-radius: 7px;
            }}
        """

    # =========================================================
    # 6. Tab 5: 智能导航 (Nav)
    # =========================================================
    NAV_BTN_MAP = """
        QPushButton { 
            background-color: #ef6c00; color: white; font-weight: bold; font-size: 15px; border-radius: 8px; border: 1px solid #f57c00;
        }
        QPushButton:hover { background-color: #f57c00; }
    """

    NAV_BTN_RUN = """
        QPushButton { 
            background-color: #1565c0; color: white; font-weight: bold; font-size: 15px; border-radius: 8px; border: 1px solid #1e88e5;
        }
        QPushButton:hover { background-color: #1e88e5; }
    """

    NAV_BTN_RELOC_PURPLE = """
        QPushButton { background-color: #7b1fa2; color: white; border-radius: 4px; border: 1px solid #8e24aa; }
        QPushButton:hover { background-color: #8e24aa; }
    """
    # [新增] 亮绿色确认按钮 (用于重定位)
    BTN_CONFIRM_GREEN = """
        QPushButton { 
            background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #00e676, stop:1 #00c853);
            color: #003300; font-weight: 900; font-size: 15px;
            border-radius: 6px; border: 1px solid #00e676;
        }
        QPushButton:hover { background-color: #69f0ae; color: black; }
        QPushButton:pressed { background-color: #00c853; }
    """
    
    # =========================================================
    # 7. Tab 6: 灵巧手 (Hand)
    # =========================================================
    HAND_CTRL_GROUP = "QGroupBox { border: 1px solid #00e5ff; margin-top: 10px; color: #00e5ff; font-weight: bold;} QGroupBox::title { subcontrol-origin: margin; left: 10px; }"
    HAND_FEED_GROUP = "QGroupBox { border: 1px solid #ffea00; margin-top: 10px; color: #ffea00; font-weight: bold;} QGroupBox::title { subcontrol-origin: margin; left: 10px; }"

    @staticmethod
    def get_hand_btn_style(color):
        return f"QPushButton {{ background-color: {color}; color: white; font-size: 16px; font-weight: bold; border-radius: 5px; }} QPushButton:pressed {{ border: 2px solid white; }}"

    # =========================================================
    # 8. Tab 7: 语音助手 (Voice)
    # =========================================================
    VOICE_BROWSER = """
        QTextBrowser {
            background-color: #1e1e1e; /* 深色背景 */
            border: 1px solid #333;
            border-radius: 8px;
            padding: 15px;
            font-family: "Microsoft YaHei", sans-serif;
        }
    """

    VOICE_INPUT_FRAME = """
        QFrame { 
            background-color: #252526; 
            border-top: 1px solid #333; 
            border-radius: 0 0 8px 8px;
        }
    """

    VOICE_INPUT_EDIT = """
        QLineEdit {
            background-color: #333; 
            color: #eee; 
            border: 1px solid #444; 
            border-radius: 20px; 
            padding: 8px 15px; 
            font-size: 14px;
        }
        QLineEdit:focus { border: 1px solid #00e5ff; }
    """

    VOICE_BTN_SEND = """
        QPushButton {
            background-color: #00c853; 
            color: white; 
            font-weight: bold; 
            border-radius: 18px;
            font-size: 13px;
        }
        QPushButton:hover { background-color: #00e676; }
        QPushButton:pressed { background-color: #009624; }
        QPushButton:disabled { background-color: #444; color: #888; }
    """
