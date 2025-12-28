import React, { createContext, useState, useEffect, useContext, ReactNode } from 'react';

// Types
interface UserProfile {
  id: string;
  email: string;
  // Software Background
  programming_experience: string;
  ros_experience: string;
  linux_familiarity: string;
  // Hardware Background
  hardware_experience: string;
  electronics_knowledge: string;
  robotics_projects: string;
  // Learning Goals
  learning_goal: string;
  // Metadata
  created_at: string;
  updated_at: string;
}

interface ProfileUpdateData {
  programming_experience?: string;
  ros_experience?: string;
  linux_familiarity?: string;
  hardware_experience?: string;
  electronics_knowledge?: string;
  robotics_projects?: string;
  learning_goal?: string;
}

interface AuthContextType {
  user: UserProfile | null;
  token: string | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  signin: (email: string, password: string) => Promise<void>;
  signup: (
    email: string,
    password: string,
    programming_experience: string,
    ros_experience?: string,
    linux_familiarity?: string,
    hardware_experience?: string,
    electronics_knowledge?: string,
    robotics_projects?: string,
    learning_goal?: string
  ) => Promise<void>;
  signout: () => void;
  updateProfile: (data: ProfileUpdateData) => Promise<void>;
  error: string | null;
  clearError: () => void;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

// API Base URL (same as ChatBot API)
const API_BASE_URL = typeof window !== 'undefined' && window.location.hostname !== 'localhost'
  ? 'https://rameesha12123214-hackathone.hf.space'
  : 'http://localhost:8001';

// API Functions
async function signinAPI(email: string, password: string) {
  const response = await fetch(`${API_BASE_URL}/auth/signin`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ email, password })
  });

  if (!response.ok) {
    const error = await response.json();
    throw new Error(error.detail || 'Signin failed');
  }

  return response.json();
}

async function signupAPI(
  email: string,
  password: string,
  programming_experience: string,
  ros_experience?: string,
  linux_familiarity?: string,
  hardware_experience?: string,
  electronics_knowledge?: string,
  robotics_projects?: string,
  learning_goal?: string
) {
  const response = await fetch(`${API_BASE_URL}/auth/signup`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      email,
      password,
      programming_experience,
      ros_experience,
      linux_familiarity,
      hardware_experience,
      electronics_knowledge,
      robotics_projects,
      learning_goal
    })
  });

  if (!response.ok) {
    const error = await response.json();
    throw new Error(error.detail || 'Signup failed');
  }

  return response.json();
}

async function getProfileAPI(token: string) {
  const response = await fetch(`${API_BASE_URL}/auth/me`, {
    headers: { 'Authorization': `Bearer ${token}` }
  });

  if (!response.ok) {
    throw new Error('Failed to get profile');
  }

  return response.json();
}

async function updateProfileAPI(token: string, data: ProfileUpdateData) {
  const response = await fetch(`${API_BASE_URL}/auth/me`, {
    method: 'PUT',
    headers: {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${token}`
    },
    body: JSON.stringify(data)
  });

  if (!response.ok) {
    const error = await response.json();
    throw new Error(error.detail || 'Profile update failed');
  }

  return response.json();
}

// Provider Component
export const AuthProvider: React.FC<{ children: ReactNode }> = ({ children }) => {
  const [user, setUser] = useState<UserProfile | null>(null);
  const [token, setToken] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  // Check for existing token on mount
  useEffect(() => {
    const storedToken = localStorage.getItem('auth_token');
    if (storedToken) {
      getProfileAPI(storedToken)
        .then(profile => {
          setUser(profile);
          setToken(storedToken);
        })
        .catch(() => {
          // Invalid token, clear it
          localStorage.removeItem('auth_token');
        })
        .finally(() => {
          setIsLoading(false);
        });
    } else {
      setIsLoading(false);
    }
  }, []);

  const signin = async (email: string, password: string) => {
    try {
      setError(null);
      const response = await signinAPI(email, password);
      localStorage.setItem('auth_token', response.access_token);
      setToken(response.access_token);
      setUser(response.user);
    } catch (err: any) {
      setError(err.message);
      throw err;
    }
  };

  const signup = async (
    email: string,
    password: string,
    programming_experience: string,
    ros_experience: string = 'None',
    linux_familiarity: string = 'Beginner',
    hardware_experience: string = 'None',
    electronics_knowledge: string = 'None',
    robotics_projects: string = 'None',
    learning_goal: string = 'General Learning'
  ) => {
    try {
      setError(null);
      const response = await signupAPI(
        email,
        password,
        programming_experience,
        ros_experience,
        linux_familiarity,
        hardware_experience,
        electronics_knowledge,
        robotics_projects,
        learning_goal
      );
      localStorage.setItem('auth_token', response.access_token);
      setToken(response.access_token);
      setUser(response.user);
    } catch (err: any) {
      setError(err.message);
      throw err;
    }
  };

  const signout = () => {
    localStorage.removeItem('auth_token');
    setToken(null);
    setUser(null);
    setError(null);
  };

  const updateProfile = async (data: ProfileUpdateData) => {
    if (!token) throw new Error('Not authenticated');

    try {
      setError(null);
      const updatedUser = await updateProfileAPI(token, data);
      setUser(updatedUser);
    } catch (err: any) {
      setError(err.message);
      throw err;
    }
  };

  const clearError = () => setError(null);

  return (
    <AuthContext.Provider value={{
      user,
      token,
      isAuthenticated: !!user,
      isLoading,
      signin,
      signup,
      signout,
      updateProfile,
      error,
      clearError
    }}>
      {children}
    </AuthContext.Provider>
  );
};

// Custom Hook
export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within AuthProvider');
  }
  return context;
};
