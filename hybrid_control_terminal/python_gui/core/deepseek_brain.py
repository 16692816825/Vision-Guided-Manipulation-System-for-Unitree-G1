# core/deepseek_brain.py
from openai import OpenAI

class G1DeepSeekBrain:
    def __init__(self, api_key):
        """
        初始化 DeepSeek 客户端
        """
        self.client = None
        self.ready = False
        
        try:
            # DeepSeek 兼容 OpenAI SDK
            self.client = OpenAI(
                api_key=api_key,
                base_url="https://api.deepseek.com"
            )
            self.ready = True
            print("[Brain] DeepSeek client initialized.")
        except Exception as e:
            print(f"[Brain] Init Failed: {e}")

        # 系统人设
        self.system_prompt = (
            "你是一个名为'G1'的实体机器人。请用简短、活泼的口语化中文回答(30字以内)。"
            "不要总是做动作。只有当用户明确要求你做动作（如'挥手'、'蹲下'）时，"
            "才在回答末尾加上标记 [ACTION:WAVE] 或 [ACTION:SQUAT]。"
            "如果是普通聊天，只回复文字即可。"
        )
        
        # 简单的对话历史记忆 (只记最近几轮，防止 Token 消耗过大)
        self.history = [{"role": "system", "content": self.system_prompt}]

    def process(self, user_text):
        """
        输入: 用户文本
        输出: (回复文本, 动作标记)
        """
        if not self.ready:
            return "API Key 未配置或连接失败", None

        # 1. 更新历史
        self.history.append({"role": "user", "content": user_text})
        
        try:
            # 2. 调用 API
            response = self.client.chat.completions.create(
                model="deepseek-chat",
                messages=self.history,
                stream=False
            )
            
            raw_reply = response.choices[0].message.content
            
            # 3. 更新历史 (存入 AI 回复)
            self.history.append({"role": "assistant", "content": raw_reply})
            
            # 4. 解析动作标记
            action = None
            clean_text = raw_reply
            
            if "[ACTION:" in raw_reply:
                start = raw_reply.find("[ACTION:")
                end = raw_reply.find("]", start)
                if end != -1:
                    action = raw_reply[start+8 : end] # 提取 WAVE 等
                    # 去掉标记，只朗读文字
                    clean_text = raw_reply[:start] + raw_reply[end+1:]
            
            return clean_text, action

        except Exception as e:
            print(f"[DeepSeek Error] {e}")
            return "大脑短路了，请检查网络。", None