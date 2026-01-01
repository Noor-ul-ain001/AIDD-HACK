import app from "../../app.js";
export default async (req, context) => {
    return app.fetch(req, {}, context);
};
export const config = {
    path: "/*"
};
