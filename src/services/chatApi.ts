import type { ChatResponse, UserBackground, ChatMessage } from '../types/chat';

const API_BASE_URL = 'https://ubushra-chatkit-chatbot.hf.space';
const API_TIMEOUT = 90000; // 90 seconds (RAG can be slow)

// Generate a unique thread ID for the session
const getOrCreateThreadId = (): string => {
  if (typeof window !== 'undefined') {
    let threadId = sessionStorage.getItem('chatkit_thread_id');
    if (!threadId) {
      threadId = `thread_${Date.now().toString(36)}_${Math.random().toString(36).substr(2, 8)}`;
      sessionStorage.setItem('chatkit_thread_id', threadId);
    }
    return threadId;
  }
  return `thread_${Date.now().toString(36)}`;
};

export async function sendMessage(
  message: string,
  userContext: UserBackground | null,
  conversationHistory: ChatMessage[] = []
): Promise<ChatResponse> {
  try {
    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), API_TIMEOUT);

    const threadId = getOrCreateThreadId();

    // Correct ChatKit SDK format discovered through testing
    const requestBody = {
      type: "threads.add_user_message",
      params: {
        thread_id: threadId,
        input: {
          content: [
            {
              type: "input_text",
              text: message
            }
          ],
          attachments: [],
          inference_options: {}
        }
      }
    };

    console.log('Sending ChatKit request:', JSON.stringify(requestBody, null, 2));

    const response = await fetch(`${API_BASE_URL}/chatkit`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Accept': 'text/event-stream',
      },
      body: JSON.stringify(requestBody),
      signal: controller.signal,
    });

    clearTimeout(timeoutId);

    if (!response.ok) {
      const errorText = await response.text();
      console.error('API Error Response:', errorText);
      throw new Error(`API Error: ${response.status}`);
    }

    // ChatKit returns Server-Sent Events (SSE) stream
    const responseText = await response.text();
    console.log('Raw API Response:', responseText);

    // Parse SSE response to extract assistant message
    const lines = responseText.split('\n');
    let assistantResponse = '';

    for (const line of lines) {
      if (line.startsWith('data: ')) {
        try {
          const data = JSON.parse(line.slice(6));

          // Look for assistant_message with content
          if (data.type === 'thread.item.done' && data.item?.type === 'assistant_message') {
            const content = data.item.content;
            if (Array.isArray(content)) {
              for (const c of content) {
                if (c.type === 'output_text' && c.text) {
                  assistantResponse = c.text;
                  break;
                }
                if (c.text) {
                  assistantResponse = c.text;
                  break;
                }
              }
            }
          }

          // Also check for streaming delta
          if (data.type === 'thread.item.delta' && data.delta?.content) {
            for (const c of data.delta.content) {
              if (c.text) {
                assistantResponse += c.text;
              }
            }
          }
        } catch (e) {
          // Skip non-JSON lines
        }
      }
    }

    if (assistantResponse) {
      return { response: assistantResponse };
    }

    // Fallback: return raw response if parsing failed
    return { response: 'I received your message but could not parse the response.' };

  } catch (error) {
    console.error('Chat API Error:', error);
    if (error instanceof Error) {
      if (error.name === 'AbortError') {
        throw new Error('Request timeout. The server took too long to respond.');
      }
      throw new Error(error.message);
    }
    throw new Error('Failed to send message. Please try again.');
  }
}

export async function checkHealth(): Promise<boolean> {
  try {
    const response = await fetch(`${API_BASE_URL}/health`, {
      method: 'GET',
      signal: AbortSignal.timeout(5000),
    });
    return response.ok;
  } catch (error) {
    return false;
  }
}

export function clearThreadId(): void {
  if (typeof window !== 'undefined') {
    sessionStorage.removeItem('chatkit_thread_id');
  }
}
