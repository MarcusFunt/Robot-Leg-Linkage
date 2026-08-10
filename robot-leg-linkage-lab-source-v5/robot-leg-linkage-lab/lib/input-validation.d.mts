export type NumericValidationRules = { min?: number; max?: number; integer?: boolean; validate?: (value: number) => string | null };
export function validateNumericValue(value: number, rules?: NumericValidationRules): string | null;
export function validateConfig(config: object): Record<string, string>;
