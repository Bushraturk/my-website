import { betterAuth } from "better-auth";

export const auth = betterAuth({
  database: {
    provider: "sqlite",
    url: process.env.DATABASE_URL || "./db.sqlite",
  },
  socialProviders: {
    // Add social providers if needed
  },
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false,
  },
  user: {
    // Custom fields for user background
    additionalFields: {
      programmingExperience: {
        type: "string",
        required: false,
      },
      hardwareExperience: {
        type: "string",
        required: false,
      },
      roboticsBackground: {
        type: "string",
        required: false,
      },
      interests: {
        type: "string", // Will store as comma-separated values
        required: false,
      },
    },
  },
});