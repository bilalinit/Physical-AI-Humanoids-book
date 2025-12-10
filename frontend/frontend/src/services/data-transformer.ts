import { ChatMessage } from './chatkit-api';

// Define ChatKit-specific types that might be used
interface ChatKitMessage {
  id: string;
  text: string;
  sender: {
    id: string;
    name?: string;
  };
  createdAt: string;
  role: 'user' | 'assistant';
  customData?: Record<string, any>;
}

interface ChatKitThread {
  id: string;
  name?: string;
  createdAt: string;
  updatedAt: string;
  customData?: Record<string, any>;
}

// Define our internal data structures
interface InternalMessage {
  id: string;
  content: string;
  role: 'user' | 'assistant';
  timestamp: string;
  sources?: Array<{
    title: string;
    url: string;
    content?: string;
  }>;
}

interface InternalThread {
  id: string;
  createdAt: string;
  updatedAt: string;
  title?: string;
  messageCount: number;
}

class DataTransformer {
  // Transform API response message to ChatKit format
  static toChatKitMessage(apiMessage: ChatMessage): ChatKitMessage {
    return {
      id: apiMessage.id,
      text: apiMessage.content,
      sender: {
        id: apiMessage.role,
        name: apiMessage.role === 'user' ? 'User' : 'Assistant'
      },
      createdAt: apiMessage.timestamp,
      role: apiMessage.role,
      customData: {
        sources: apiMessage.sources,
        originalMessage: apiMessage
      }
    };
  }

  // Transform ChatKit message to our internal API format
  static fromChatKitMessage(chatKitMessage: ChatKitMessage): InternalMessage {
    return {
      id: chatKitMessage.id,
      content: chatKitMessage.text,
      role: chatKitMessage.role,
      timestamp: chatKitMessage.createdAt,
      sources: chatKitMessage.customData?.sources
    };
  }

  // Transform API thread to ChatKit format
  static toChatKitThread(apiThread: any): ChatKitThread {
    return {
      id: apiThread.id,
      name: apiThread.title,
      createdAt: apiThread.createdAt,
      updatedAt: apiThread.updatedAt,
      customData: {
        messageCount: apiThread.messageCount,
        originalThread: apiThread
      }
    };
  }

  // Transform ChatKit thread to our internal format
  static fromChatKitThread(chatKitThread: ChatKitThread): InternalThread {
    return {
      id: chatKitThread.id,
      createdAt: chatKitThread.createdAt,
      updatedAt: chatKitThread.updatedAt,
      title: chatKitThread.name,
      messageCount: chatKitThread.customData?.messageCount || 0
    };
  }

  // Transform our API response to ChatKit-compatible format
  static transformApiResponse(response: any): any {
    if (response.thread && response.thread.messages) {
      // Transform thread with messages
      return {
        ...response,
        thread: {
          ...response.thread,
          messages: response.thread.messages.map(this.toChatKitMessage)
        }
      };
    } else if (response.threads && Array.isArray(response.threads)) {
      // Transform list of threads
      return {
        ...response,
        threads: response.threads.map(this.toChatKitThread)
      };
    } else if (response.message) {
      // Transform single message response
      return {
        ...response,
        message: this.toChatKitMessage(response.message)
      };
    }
    return response;
  }

  // Transform ChatKit data to our API format
  static transformToApiFormat(data: any): any {
    if (data.messages && Array.isArray(data.messages)) {
      // Transform list of messages
      return {
        ...data,
        messages: data.messages.map(this.fromChatKitMessage)
      };
    } else if (data.text && data.sender) {
      // Transform single message
      return {
        content: data.text,
        role: data.sender.id as 'user' | 'assistant',
        timestamp: data.createdAt
      };
    }
    return data;
  }
}

export default DataTransformer;