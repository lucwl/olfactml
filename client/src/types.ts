
export interface LoggingPayload {
  idx: number
  f_idx: number
  pos: number
  p_temp: number
  h_dur: number
  t: number
  p: number
  h: number
  gas: number
  lbl: number
}

export interface ScoreItem {
  label: string
  score: number
}

export interface InferencePayload {
  prediction: string
  scores: Record<string, ScoreItem>
}

export interface Message<T> {
  op: number
  d: T
}

export enum Mode {
  Logging = 1,
  Inference = 2
}
