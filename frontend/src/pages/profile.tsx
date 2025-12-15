import React from 'react';
import Layout from '@theme/Layout';
import Profile from '@site/src/components/Auth/Profile';
import { AuthProvider } from '@site/src/hooks/useAuth';

export default function ProfilePage() {
    return (
        <Layout
            title="My Profile"
            description="View and edit your profile"
        >
            <main style={{
                minHeight: 'calc(100vh - 60px)',
                display: 'flex',
                alignItems: 'flex-start',
                justifyContent: 'center',
                padding: '2rem'
            }}>
                <div style={{
                    maxWidth: '600px',
                    width: '100%',
                    backgroundColor: 'var(--ifm-background-surface-color)',
                    padding: '2rem',
                    borderRadius: '8px',
                    boxShadow: '0 2px 8px rgba(0, 0, 0, 0.1)'
                }}>
                    <AuthProvider>
                        <Profile />
                    </AuthProvider>

                    {/* Navigation links */}
                    <div style={{
                        marginTop: '20px',
                        textAlign: 'center',
                        borderTop: '1px solid var(--ifm-color-emphasis-300)',
                        paddingTop: '20px'
                    }}>
                        <a
                            href="/"
                            style={{
                                color: 'var(--ifm-link-color)',
                                marginRight: '20px'
                            }}
                        >
                            ← Back to Home
                        </a>
                        <button
                            onClick={() => {
                                localStorage.removeItem('auth_session');
                                localStorage.removeItem('chatkit-thread-id');
                                window.location.href = '/signin';
                            }}
                            style={{
                                padding: '8px 16px',
                                backgroundColor: '#dc3545',
                                color: 'white',
                                border: 'none',
                                borderRadius: '4px',
                                cursor: 'pointer'
                            }}
                        >
                            Sign Out
                        </button>
                    </div>
                </div>
            </main>
        </Layout>
    );
}
