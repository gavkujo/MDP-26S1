// Test component to check backend connection
import { useState } from 'react';
import BaseAPI, { methodType } from './BaseAPI';

export default function ConnectionTest() {
  const [connectionStatus, setConnectionStatus] = useState('idle');
  const [backendInfo, setBackendInfo] = useState(null);

  const testConnection = async () => {
    setConnectionStatus('testing');
    try {
      const response = await BaseAPI(methodType.GET, '/test');
      if (response.status === 200 && response.data.message === "Backend is running") {
        setConnectionStatus('success');
        setBackendInfo(response.data);
      } else {
        throw new Error('Unexpected response from backend');
      }
    } catch (error) {
      console.error('Connection test failed:', error);
      setConnectionStatus('failed');
      setBackendInfo(error.message || 'Connection failed');
    }
  };

  return (
    <div className="bg-white rounded-2xl shadow-xl border border-gray-100 p-6 mb-6">
      <div className="flex items-center justify-between mb-4">
        <h3 className="text-lg font-semibold text-gray-800">Backend Connection</h3>
        <div className={`px-3 py-1 rounded-full text-sm font-medium ${
          connectionStatus === 'success' ? 'bg-green-100 text-green-800' :
          connectionStatus === 'failed' ? 'bg-red-100 text-red-800' :
          connectionStatus === 'testing' ? 'bg-yellow-100 text-yellow-800' :
          'bg-gray-100 text-gray-800'
        }`}>
          {connectionStatus === 'success' ? '✅ Connected' :
           connectionStatus === 'failed' ? '❌ Failed' :
           connectionStatus === 'testing' ? '🔄 Testing...' :
           '⚪ Not tested'}
        </div>
      </div>
      
      <div className="text-sm text-gray-600 mb-4">
        Current backend URL: <code className="bg-gray-100 px-2 py-1 rounded">{process.env.NEXT_PUBLIC_API_URL || 'http://localhost:5000/'}</code>
      </div>
      
      <button
        onClick={testConnection}
        disabled={connectionStatus === 'testing'}
        className="bg-blue-500 text-white px-4 py-2 rounded-lg hover:bg-blue-600 disabled:opacity-50 disabled:cursor-not-allowed"
      >
        {connectionStatus === 'testing' ? 'Testing...' : 'Test Connection'}
      </button>
      
      {backendInfo && (
        <div className="mt-4 p-3 bg-gray-50 rounded-lg">
          <pre className="text-xs overflow-auto">{JSON.stringify(backendInfo, null, 2)}</pre>
        </div>
      )}
    </div>
  );
}
