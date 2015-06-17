﻿using System;
using Windows.Storage;

namespace AdafruitSensor
{
    /// <summary>
    ///  Class for managing app settings
    /// </summary>
    public sealed class AppSettings
    {
        // Our settings
        ApplicationDataContainer localSettings;

        // The key names of our settings
        const string SettingsSetKeyname = "settingsset";
        const string ServicebusNamespaceKeyname = "galileowiniot";
        const string EventHubNameKeyname = "galileosensorevents";
        const string KeyNameKeyname = "ManagePolicy";
        const string KeyKeyname = "MjfjIi+ddeHifOLHRLjpmS2n5mKG+R1xrWqzgUJYty0=";
        const string DisplayNameKeyname = "raspberrypiII";
        const string OrganizationKeyname = "e-consystems";
        const string LocationKeyname = "India";

        // The default value of our settings
        const bool SettingsSetDefault = true;
        const string ServicebusNamespaceDefault = "galileowiniot";
        const string EventHubNameDefault = "galileosensorevents";
        const string KeyNameDefault = "ManagePolicy";
        const string KeyDefault = "MjfjIi+ddeHifOLHRLjpmS2n5mKG+R1xrWqzgUJYty0=";
        const string DisplayNameDefault = "raspberrypiII";
        const string OrganizationDefault = "e-consystems";
        const string LocationDefault = "India";

        /// <summary>
        /// Constructor that gets the application settings.
        /// </summary>
        public AppSettings()
        {
            // Get the settings for this application.
            localSettings = ApplicationData.Current.LocalSettings;
        }

        /// <summary>
        /// Update a setting value for our application. If the setting does not
        /// exist, then add the setting.
        /// </summary>
        /// <param name="Key"></param>
        /// <param name="value"></param>
        /// <returns></returns>
        public bool AddOrUpdateValue(string Key, Object kvalue)
        {
            bool valueChanged = false;

            // If the key exists
            if (localSettings.Values.ContainsKey(Key))
            {
                // If the value has changed
                if (localSettings.Values[Key] != kvalue)
                {
                    // Store the new value
                    localSettings.Values[Key] = kvalue;
                    valueChanged = true;
                }
            }
            // Otherwise create the key.
            else
            {
                localSettings.Values.Add(Key, kvalue);
                valueChanged = true;
            }
            return valueChanged;
        }

        /// <summary>
        /// Get the current value of the setting, or if it is not found, set the 
        /// setting to the default setting.
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="Key"></param>
        /// <param name="defaultValue"></param>
        /// <returns></returns>
        public bool GetValueOrDefaultBool (string Key, bool defaultValue)
        {
            bool value;

            // If the key exists, retrieve the value.
            if (localSettings.Values.ContainsKey(Key))
            {
                value = (bool)localSettings.Values[Key];
            }
            // Otherwise, use the default value.
            else
            {
                value = defaultValue;
            }
            return value;
        }

        /// Get the current value of the setting, or if it is not found, set the 
        /// setting to the default setting.
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="Key"></param>
        /// <param name="defaultValue"></param>
        /// <returns></returns>
        public string GetValueOrDefaultString(string Key, string defaultValue)
        {
            string value;

            // If the key exists, retrieve the value.
            if (localSettings.Values.ContainsKey(Key))
            {
                value = (string)localSettings.Values[Key];
            }
            // Otherwise, use the default value.
            else
            {
                value = defaultValue;
            }
            return value;
        }


        /// <summary>
        /// Save the settings.
        /// </summary>
        public void Save()
        {
            // keeping the below in case we want to use this code on a Windows Phone 8 device.
            // With universal Windows Apps, this is no longer necessary as settings are saved automatically
            //            settings.Save();
        }


        /// <summary>
        /// Property to get and set a Username Setting Key.
        /// </summary>
        public bool SettingsSet
        {
            get
            {
                return GetValueOrDefaultBool(SettingsSetKeyname, SettingsSetDefault);
            }
            set
            {
                if (AddOrUpdateValue(SettingsSetKeyname, value))
                {
                    Save();
                }
            }
        }

        /// <summary>
        /// Property to get and set a Username Setting Key.
        /// </summary>
        public string ServicebusNamespace
        {
            get
            {
                return GetValueOrDefaultString(ServicebusNamespaceKeyname, ServicebusNamespaceDefault);
            }
            set
            {
                if (AddOrUpdateValue(ServicebusNamespaceKeyname, value))
                {
                    Save();
                }
            }
        }

        /// <summary>
        /// Property to get and set a Username Setting Key.
        /// </summary>
        public string EventHubName
        {
            get
            {
                return GetValueOrDefaultString(EventHubNameKeyname, EventHubNameDefault);
            }
            set
            {
                if (AddOrUpdateValue(EventHubNameKeyname, value))
                {
                    Save();
                }
            }
        }
        /// <summary>
        /// Property to get and set a Username Setting Key.
        /// </summary>
        public string KeyName
        {
            get
            {
                return GetValueOrDefaultString(KeyNameKeyname, KeyNameDefault);
            }
            set
            {
                if (AddOrUpdateValue(KeyNameKeyname, value))
                {
                    Save();
                }
            }
        }
        /// <summary>
        /// Property to get and set a Username Setting Key.
        /// </summary>
        public string Key
        {
            get
            {
                return GetValueOrDefaultString(KeyKeyname, KeyDefault);
            }
            set
            {
                if (AddOrUpdateValue(KeyKeyname, value))
                {
                    Save();
                }
            }
        }
        /// <summary>
        /// Property to get and set a Username Setting Key.
        /// </summary>
        public string DisplayName
        {
            get
            {
                return GetValueOrDefaultString(DisplayNameKeyname, DisplayNameDefault);
            }
            set
            {
                if (AddOrUpdateValue(DisplayNameKeyname, value))
                {
                    Save();
                }
            }
        }
        /// <summary>
        /// Property to get and set a Username Setting Key.
        /// </summary>
        public string Organization
        {
            get
            {
                return GetValueOrDefaultString(OrganizationKeyname, OrganizationDefault);
            }
            set
            {
                if (AddOrUpdateValue(OrganizationKeyname, value))
                {
                    Save();
                }
            }
        }
        /// <summary>
        /// Property to get and set a Username Setting Key.
        /// </summary>
        public string Location
        {
            get
            {
                return GetValueOrDefaultString(LocationKeyname, LocationDefault);
            }
            set
            {
                if (AddOrUpdateValue(LocationKeyname, value))
                {
                    Save();
                }
            }
        }

    }
}
